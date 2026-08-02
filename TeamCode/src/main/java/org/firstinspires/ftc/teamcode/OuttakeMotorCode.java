package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Outtake Motor Test")
public class OuttakeMotorCode extends LinearOpMode {

    private DcMotorEx outtake_motor;

    // goBILDA 5203 Series Yellow Jacket, 6000 RPM (no gearbox) — 28 ticks per revolution
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 5500;
    private static final double TARGET_TICKS_PER_SEC = (TARGET_RPM * TICKS_PER_REV) / 60.0;

    private boolean outtakeOn = false;
    private boolean bPressedLast = false;

    @Override
    public void runOpMode() {
        outtake_motor = hardwareMap.get(DcMotorEx.class, "outtake_motor");

        outtake_motor.setDirection(DcMotor.Direction.FORWARD);
        outtake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            boolean bPressed = gamepad1.b;
            if (bPressed && !bPressedLast) {
                outtakeOn = !outtakeOn;
            }
            bPressedLast = bPressed;

            if (outtakeOn) {
                outtake_motor.setVelocity(TARGET_TICKS_PER_SEC);
            } else {
                outtake_motor.setVelocity(0);
            }

            telemetry.addData("Outtake On", outtakeOn);
            telemetry.addData("Target Velocity (ticks/s)", TARGET_TICKS_PER_SEC);
            telemetry.addData("Current Velocity (ticks/s)", outtake_motor.getVelocity());
            telemetry.update();
        }
    }
}