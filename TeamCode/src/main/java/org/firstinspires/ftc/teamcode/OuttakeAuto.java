package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeAuto {
    private DcMotor outtake_motor;

    // Full power = the motor's max free-run speed.
    // For a goBILDA 5203 Yellow Jacket (13.7:1 ratio), that's ~435 RPM at 12V.
    private static final double INTAKE_POWER = 1.0;

    public OuttakeAuto(HardwareMap hardwareMap) {
        outtake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
    }

    public void shootOut(int rpm) {
        outtake_motor.setPower(INTAKE_POWER);
    }

    public void stopShoot() {
        outtake_motor.setPower(0);
    }
}