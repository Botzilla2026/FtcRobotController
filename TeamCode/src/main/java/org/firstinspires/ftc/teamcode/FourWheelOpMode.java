package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Full Robot TeleOp — Mecanum Drive + Intake + Control + Outtake
 *
 * Drive Control:
 *   Right Joystick -> Forward/Back + Strafe
 *   Left  Joystick -> Points in the direction the robot should face (absolute heading control)
 *
 * Mechanism Control:
 *   X -> Intake toggle (435 RPM on/off)
 *   Y -> Control motor toggle (30 RPM <-> 400 RPM)
 *   B -> Outtake toggle (5500 RPM on/off, reversed direction)
 */
@TeleOp(name = "Full Robot TeleOp", group = "TeleOp")
public class FourWheelOpMode extends LinearOpMode {

    // ------------------- Drivetrain -------------------
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

    private static final double MAX_SPEED = 1.0;
    private static final double DEADZONE  = 0.05;

    // ------------------- Heading control (left stick) -------------------
    private IMU imu;
    private static final double HEADING_KP = 0.8; // tune: proportional gain, target-current error -> rotate power
    private static final double HEADING_STICK_DEADZONE = 0.15; // ignore small left-stick nudges
    private double targetHeadingRad = 0.0; // holds last commanded heading when stick is released

    // ------------------- Mechanisms -------------------
    private DcMotorEx intake_motor;
    private DcMotorEx control_motor;
    private DcMotorEx outtake_motor;

    // Intake: goBILDA 5203 19.2:1, 435 RPM target
    private static final double INTAKE_TICKS_PER_REV = 384.5;
    private static final double INTAKE_TARGET_RPM = 435;
    private static final double INTAKE_TICKS_PER_SEC =
            (INTAKE_TARGET_RPM * INTAKE_TICKS_PER_REV) / 60.0;
    private boolean intakeOn = false;
    private boolean xPressedLast = false;

    // Control: goBILDA 5203 19.2:1, toggles between 30 RPM and 400 RPM
    private static final double CONTROL_TICKS_PER_REV = 384.5;
    private static final double CONTROL_RPM_LOW  = 30;
    private static final double CONTROL_RPM_HIGH = 400;
    private static final double CONTROL_TICKS_PER_SEC_LOW =
            (CONTROL_RPM_LOW * CONTROL_TICKS_PER_REV) / 60.0;
    private static final double CONTROL_TICKS_PER_SEC_HIGH =
            (CONTROL_RPM_HIGH * CONTROL_TICKS_PER_REV) / 60.0;
    private boolean controlAtHigh = false; // false = 30 RPM, true = 400 RPM
    private boolean yPressedLast = false;

    // Outtake: goBILDA 5203 Yellow Jacket (no gearbox, 6000 RPM), target 5500 RPM
    private static final double OUTTAKE_TICKS_PER_REV = 28.0;
    private static final double OUTTAKE_TARGET_RPM = 5500;
    private static final double OUTTAKE_TICKS_PER_SEC =
            (OUTTAKE_TARGET_RPM * OUTTAKE_TICKS_PER_REV) / 60.0;
    private boolean outtakeOn = false;
    private boolean bPressedLast = false;

    @Override
    public void runOpMode() {

        // ---------------- Hardware Map: Drivetrain ----------------
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftBackMotor   = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightBackMotor  = hardwareMap.get(DcMotor.class, "right_back_motor");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---------------- Hardware Map: IMU ----------------
        // Adjust logoFacingDirection / usbFacingDirection to match how your Control/Expansion Hub
        // is physically mounted on the robot.
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParameters);
        imu.resetYaw();

        // ---------------- Hardware Map: Mechanisms ----------------
        intake_motor  = hardwareMap.get(DcMotorEx.class, "intake_motor");
        control_motor = hardwareMap.get(DcMotorEx.class, "control_motor");
        outtake_motor = hardwareMap.get(DcMotorEx.class, "outtake_motor");

        intake_motor.setDirection(DcMotor.Direction.FORWARD);
        control_motor.setDirection(DcMotor.Direction.FORWARD);
        outtake_motor.setDirection(DcMotor.Direction.REVERSE);

        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        control_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        control_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        control_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized - waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ============= DRIVETRAIN: translation (right stick) =============
            double rawDrive  = -gamepad1.right_stick_y;
            double rawStrafe =  gamepad1.right_stick_x;

            double drive, strafe;

            if (Math.abs(rawDrive) < DEADZONE) {
                drive = 0.0;
            } else {
                drive = (rawDrive - Math.signum(rawDrive) * DEADZONE) / (1.0 - DEADZONE);
            }

            if (Math.abs(rawStrafe) < DEADZONE) {
                strafe = 0.0;
            } else {
                strafe = (rawStrafe - Math.signum(rawStrafe) * DEADZONE) / (1.0 - DEADZONE);
            }

            // ============= DRIVETRAIN: heading (left stick) =============
            double stickX = gamepad1.left_stick_x;
            double stickY = -gamepad1.left_stick_y; // up on stick = forward
            double stickMagnitude = Math.hypot(stickX, stickY);

            if (stickMagnitude > HEADING_STICK_DEADZONE) {
                // 0 rad = forward, positive = clockwise (to the right)
                targetHeadingRad = Math.atan2(stickX, stickY);
            }
            // else: keep holding the last commanded heading

            double currentHeadingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double headingError = targetHeadingRad - currentHeadingRad;
            while (headingError > Math.PI)  headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            double rotate = Range.clip(headingError * HEADING_KP, -MAX_SPEED, MAX_SPEED);

            double leftFrontPower  = drive + strafe + rotate;
            double rightFrontPower = drive - strafe - rotate;
            double leftBackPower   = drive - strafe + rotate;
            double rightBackPower  = drive + strafe - rotate;

            double maxPower = Math.max(
                    Math.max(Math.abs(leftFrontPower),  Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower),   Math.abs(rightBackPower))
            );
            if (maxPower > MAX_SPEED) {
                leftFrontPower  /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower   /= maxPower;
                rightBackPower  /= maxPower;
            }

            leftFrontPower  = Range.clip(leftFrontPower,  -MAX_SPEED, MAX_SPEED);
            rightFrontPower = Range.clip(rightFrontPower, -MAX_SPEED, MAX_SPEED);
            leftBackPower   = Range.clip(leftBackPower,   -MAX_SPEED, MAX_SPEED);
            rightBackPower  = Range.clip(rightBackPower,  -MAX_SPEED, MAX_SPEED);

            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);

            // ============= INTAKE (X toggle) =============
            boolean xPressed = gamepad1.x;
            if (xPressed && !xPressedLast) {
                intakeOn = !intakeOn;
            }
            xPressedLast = xPressed;
            intake_motor.setVelocity(intakeOn ? INTAKE_TICKS_PER_SEC : 0);

            // ============= CONTROL (Y toggle: 30 RPM <-> 400 RPM) =============
            boolean yPressed = gamepad1.y;
            if (yPressed && !yPressedLast) {
                controlAtHigh = !controlAtHigh;
            }
            yPressedLast = yPressed;
            control_motor.setVelocity(controlAtHigh ? CONTROL_TICKS_PER_SEC_HIGH : CONTROL_TICKS_PER_SEC_LOW);

            // ============= OUTTAKE (B toggle) =============
            boolean bPressed = gamepad1.b;
            if (bPressed && !bPressedLast) {
                outtakeOn = !outtakeOn;
            }
            bPressedLast = bPressed;
            outtake_motor.setVelocity(outtakeOn ? OUTTAKE_TICKS_PER_SEC : 0);

            // ============= TELEMETRY =============
            telemetry.addData("-- Drivetrain -----------", "");
            telemetry.addData("Left  Front", "%.2f", leftFrontPower);
            telemetry.addData("Right Front", "%.2f", rightFrontPower);
            telemetry.addData("Left  Back",  "%.2f", leftBackPower);
            telemetry.addData("Right Back",  "%.2f", rightBackPower);
            telemetry.addData("Target Heading (deg)", "%.1f", Math.toDegrees(targetHeadingRad));
            telemetry.addData("Current Heading (deg)", "%.1f", Math.toDegrees(currentHeadingRad));
            telemetry.addData("-- Mechanisms -----------", "");
            telemetry.addData("Intake On", intakeOn);
            telemetry.addData("Control Motor", controlAtHigh ? "400 RPM" : "30 RPM");
            telemetry.addData("Outtake On", outtakeOn);
            telemetry.update();
        }

        // Stop everything on exit
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        intake_motor.setVelocity(0);
        control_motor.setVelocity(0);
        outtake_motor.setVelocity(0);
    }
}