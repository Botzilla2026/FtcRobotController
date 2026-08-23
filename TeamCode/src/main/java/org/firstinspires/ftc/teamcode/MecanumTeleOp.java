package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * MecanumTeleOp - Pedro Pathing version
 *
 * GAMEPAD 1 CONTROLS:
 *   Left Stick  Y-axis   → Forward / Backward
 *   Left Stick  X-axis   → Strafe left / right
 *   Right Stick X-axis   → Rotate left / right
 *   A (press)            → Toggle slow mode (~40% power)
 *   B (press)            → Toggle field-centric / robot-centric
 *   Right Trigger        → Reset IMU heading
 */
@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Pedro + IMU
    // -------------------------------------------------------------------------
    private Follower follower;
    private IMU imu;

    // -------------------------------------------------------------------------
    // Tuning constants
    // -------------------------------------------------------------------------
    private static final double DEADZONE             = 0.05;
    private static final double STRAFE_CORRECTION    = 0.85;
    private static final double SLOW_MODE_MULTIPLIER = 0.4;

    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private boolean slowModeOn     = false;
    private boolean fieldCentricOn = true;
    private boolean prevA          = false;
    private boolean prevB          = false;
    private boolean prevRT         = false;

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------
    private double applyCurve(double raw) {
        return raw * raw * raw;
    }

    private double processAxis(double raw) {
        return Math.abs(raw) > DEADZONE ? applyCurve(raw) : 0.0;
    }

    private boolean risingEdge(boolean current, boolean previous) {
        return current && !previous;
    }

    // =========================================================================
    @Override
    public void runOpMode() {

        // --- PEDRO INIT ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        // --- IMU INIT ---
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR)
        ));
        imu.resetYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls",
                "LS: drive | RS-X: rotate | A: slow | B: field-centric | RT: reset heading");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();
        follower.update();

        // =========================================================================
        // MAIN LOOP
        // =========================================================================
        while (opModeIsActive()) {

            follower.update();

            // --- BUTTON TOGGLES ---
            boolean currA  = gamepad1.a;
            boolean currB  = gamepad1.b;
            boolean currRT = gamepad1.right_trigger > 0.5;

            if (risingEdge(currA, prevA))   slowModeOn     = !slowModeOn;
            if (risingEdge(currB, prevB))   fieldCentricOn = !fieldCentricOn;
            if (risingEdge(currRT, prevRT)) imu.resetYaw();

            prevA  = currA;
            prevB  = currB;
            prevRT = currRT;

            // --- JOYSTICK INPUTS ---
            double axial   = processAxis(-gamepad1.left_stick_y);
            double lateral = processAxis(-gamepad1.left_stick_x) * STRAFE_CORRECTION;
            double yaw     = processAxis(-gamepad1.right_stick_x);

            // --- SLOW MODE ---
            if (slowModeOn) {
                axial   *= SLOW_MODE_MULTIPLIER;
                lateral *= SLOW_MODE_MULTIPLIER;
                yaw     *= SLOW_MODE_MULTIPLIER;
            }

            // --- FIELD-CENTRIC ROTATION ---
            if (fieldCentricOn) {
                double heading        = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotatedAxial   = axial * Math.cos(-heading) - lateral * Math.sin(-heading);
                double rotatedLateral = axial * Math.sin(-heading) + lateral * Math.cos(-heading);
                axial   = rotatedAxial;
                lateral = rotatedLateral;
            }

            // --- SEND TO PEDRO ---
            // true = robot-centric (we handle field-centric ourselves above)
            follower.setTeleOpDrive(axial, lateral, yaw, true);

            // --- TELEMETRY ---
            telemetry.addData("Status",       "Running");
            telemetry.addData("Slow Mode",     slowModeOn     ? "ON"           : "OFF");
            telemetry.addData("Drive Mode",    fieldCentricOn ? "Field-Centric" : "Robot-Centric");
            telemetry.addData("Heading (degz)", "%.1f",
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("X",             "%.2f", follower.getPose().getX());
            telemetry.addData("Y",             "%.2f", follower.getPose().getY());
            telemetry.update();

        } // end while

    } // end runOpMode

} // end class