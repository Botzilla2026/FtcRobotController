package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeAuto {
    private DcMotorEx intake_motor;

    // goBILDA 5203 series (19.2:1) — 384.5 ticks per output revolution
    private static final double TICKS_PER_REV = 384.5;
    private boolean intakeOn = false;
    public IntakeAuto(HardwareMap hardwareMap) {
        intake_motor = hardwareMap.get(DcMotorEx.class, "intake_motor");

        intake_motor.setDirection(DcMotor.Direction.FORWARD); // spins clockwise
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void takein(double TARGET_RPM) {
        double TARGET_TICKS_PER_SEC = (TARGET_RPM * TICKS_PER_REV) / 60.0;

        intake_motor.setVelocity(TARGET_TICKS_PER_SEC);
    }
    public void stoptake(){
        intake_motor.setVelocity(0);
    }
}
