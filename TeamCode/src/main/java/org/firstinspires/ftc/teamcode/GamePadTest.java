package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name= "GamePadTest")
public class GamePadTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("GP1 type", gamepad1.type());
            telemetry.addData("GP1 id", gamepad1.getGamepadId());
            telemetry.update();
        }
    }

}
