package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Control Motor Test")
public class ControlMotorCode extends LinearOpMode {

    private DcMotorEx control_motor;

    // goBILDA 5203 series (19.2:1) — 384.5 ticks per output revolution
    private static final double TICKS_PER_REV = 384.5;

    private static final double RPM_STAGE_1 = 50;
    private static final double RPM_STAGE_2 = 500;

    private static final double TICKS_PER_SEC_STAGE_1 = (RPM_STAGE_1 * TICKS_PER_REV) / 60.0;
    private static final double TICKS_PER_SEC_STAGE_2 = (RPM_STAGE_2 * TICKS_PER_REV) / 60.0;

    // 0 = stopped, 1 = 50 RPM, 2 = 500 RPM
    private int stage = 0;
    private boolean yPressedLast = false;

    @Override
    public void runOpMode() {
        control_motor = hardwareMap.get(DcMotorEx.class, "control_motor");

        control_motor.setDirection(DcMotor.Direction.FORWARD);
        control_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        control_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        control_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            boolean yPressed = gamepad1.y;
            if (yPressed && !yPressedLast) {
                stage = (stage + 1) % 3; // cycles 0 -> 1 -> 2 -> 0 ...
            }
            yPressedLast = yPressed;

            switch (stage) {
                case 1:
                    control_motor.setVelocity(TICKS_PER_SEC_STAGE_1);
                    break;
                case 2:
                    control_motor.setVelocity(TICKS_PER_SEC_STAGE_2);
                    break;
                default:
                    control_motor.setVelocity(0);
                    break;
            }

            telemetry.addData("Stage", stage);
            telemetry.addData("Target Velocity (ticks/s)",
                    stage == 1 ? TICKS_PER_SEC_STAGE_1 : (stage == 2 ? TICKS_PER_SEC_STAGE_2 : 0));
            telemetry.addData("Current Velocity (ticks/s)", control_motor.getVelocity());
            telemetry.update();
        }
    }
}
