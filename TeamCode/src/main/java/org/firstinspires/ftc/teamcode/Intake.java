package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * Intake TeleOp
 *
 * Press X (gamepad1) once to turn the intake ON at full power (counterclockwise).
 * Press X again to turn it OFF. This is a toggle, not hold-to-run.
 *
 * Hardware setup:
 *   - In the Robot Controller app's hardware configuration, name this motor
 *     "intake" and assign it to Motor Port 1 on the Expansion Hub.
 *     (Port assignment happens in that config screen, not in this code -
 *     the code just looks up the motor by the name you gave it there.)
 */
@TeleOp(name = "Intake TeleOp", group = "TeleOp")
public class Intake extends LinearOpMode {

    // Motor that drives the intake
    private DcMotor intake_motor;

    // Full power = the motor's max free-run speed.
    // For a goBILDA 5203 Yellow Jacket (13.7:1 ratio), that's ~435 RPM at 12V.
    private static final double INTAKE_POWER = 1.0;

    // Toggle state and edge-detection for the X button
    private boolean intakeOn = false;
    private boolean xPressedLast = false;

    @Override
    public void runOpMode() {

        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");

        // REVERSE makes positive power spin the motor counterclockwise.
        // If it spins the wrong way once you test it, just change this to FORWARD.
        intake_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Motor brakes when power is 0. Switch to FLOAT if you'd rather it coast.
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Intake ready. Press X to toggle intake on/off.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean xPressed = gamepad1.x;

            // Toggle only on the rising edge (the moment X is first pressed),
            // so holding it down doesn't rapidly flip the state back and forth.
            if (xPressed && !xPressedLast) {
                intakeOn = !intakeOn;
            }
            xPressedLast = xPressed;

            intake_motor.setPower(intakeOn ? INTAKE_POWER : 0.0);

            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Intake power", intake_motor.getPower());
            telemetry.update();
        }
    }
}