package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Intake Motor Test")
public class IntakeMotorCode extends LinearOpMode {

    private DcMotorEx intake_motor;

    // goBILDA 5203 series (19.2:1) — 384.5 ticks per output revolution
    private static final double TICKS_PER_REV = 384.5;
    private static final double TARGET_RPM = 435;
    private static final double TARGET_TICKS_PER_SEC = (TARGET_RPM * TICKS_PER_REV) / 60.0;

    private boolean intakeOn = false;
    private boolean xPressedLast = false;

    @Override
    public void runOpMode() {
        intake_motor = hardwareMap.get(DcMotorEx.class, "intake_motor");

        intake_motor.setDirection(DcMotor.Direction.FORWARD); // spins clockwise
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            boolean xPressed = gamepad1.x;
            if (xPressed && !xPressedLast) {
                intakeOn = !intakeOn;
            }
            xPressedLast = xPressed;

            if (intakeOn) {
                intake_motor.setVelocity(TARGET_TICKS_PER_SEC);
            } else {
                intake_motor.setVelocity(0);
            }

            telemetry.addData("Intake On", intakeOn);
            telemetry.addData("Target Velocity (ticks/s)", TARGET_TICKS_PER_SEC);
            telemetry.addData("Current Velocity (ticks/s)", intake_motor.getVelocity());
            telemetry.update();
        }
    }
}
