package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ScrimmageAutoOp", group = "Autonomous")
public class ScrimmageAutoOp extends LinearOpMode{

    ElapsedTime timer = new ElapsedTime();   // starts counting immediately
    @Override
    public void runOpMode(){
        DcMotor leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");     // Control Hub motor port 0
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);


                 // time since reset, in seconds
        waitForStart();
        timer.reset();
        while (opModeIsActive()){

            double sec = timer.seconds();
            if (sec <= 1){
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
            }
            else if(sec > 1 && sec < 1.1) {
                leftDrive.setPower(-1.0);
                rightDrive.setPower(-1.0);
            }
            else if (sec >= 1.1 && sec <= 2.0){
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
            }
            else if (sec >= 2.0 && sec <= 3){
                leftDrive.setPower(-0.5);
                rightDrive.setPower(-0.5);
            }
            else if (sec >= 3 && sec <= 3.1){
                leftDrive.setPower(1.0);
                rightDrive.setPower(1.0);
            }
            else {
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                return;
            }
        }
    }

}
