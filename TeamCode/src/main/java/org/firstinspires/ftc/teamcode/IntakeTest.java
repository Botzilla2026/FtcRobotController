package org. firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Intake Test")
public class IntakeTest extends LinearOpMode {

    private DcMotorEx intake_motor;

    // goBILDA 5203 (312 RPM) series: 537.7 ticks per revolution
    static final double TICKS_PER_REV = 537.7;
    static final double TARGET_RPM = 300;
    static final double TARGET_TICKS_PER_SEC = (TARGET_RPM / 60.0) * TICKS_PER_REV;

    @Override
    public void runOpMode() {
        intake_motor = hardwareMap.get(DcMotorEx.class, "intake_motor");

        intake_motor.setDirection(DcMotor.Direction.FORWARD); // clockwise; flip to REVERSE if it spins backward
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            intake_motor.setVelocity(TARGET_TICKS_PER_SEC); // ticks/sec, holds true 300 RPM
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Actual velocity (ticks/s)", intake_motor.getVelocity());
            telemetry.update();
        }
    }
}