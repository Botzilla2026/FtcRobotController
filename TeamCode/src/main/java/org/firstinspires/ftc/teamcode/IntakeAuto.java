package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeAuto {
    private DcMotor intake_motor;

    // Full power = the motor's max free-run speed.
    // For a goBILDA 5203 Yellow Jacket (13.7:1 ratio), that's ~435 RPM at 12V.
    private static final double INTAKE_POWER = 1.0;

    // Toggle state and edge-detection for the X button
    private boolean intakeOn = false;

}
