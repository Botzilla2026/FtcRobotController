package org.firstinspires.ftc.teamcode;
// Limelight camera function for

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;


@Autonomous(name = "April Tag", group = "Autonomous")
public class AprilTag extends LinearOpMode{
    boolean initialized = false;
    Limelight3A limelight;
    
    @Override
    public void runOpMode() {
        if (!initialized) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(1); // whichever pipeline index detects the ball
            limelight.start();
            initialized = true;
        }

        LLResult result = limelight.getLatestResult();
        waitForStart();
        while (opModeIsActive()){
            if (result != null && result.isValid()) {
                // Simple aiming angles (degrees)
                double tx = result.getTx();
                double ty = result.getTy();

                // Per-tag 3D data
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int id = tag.getFiducialId();
                    Pose3D tagInCamera = tag.getTargetPoseCameraSpace();
                    Position pos = tagInCamera.getPosition();
                    Pose3D robotInTag = tag.getRobotPoseTargetSpace();
                    YawPitchRollAngles ori = tagInCamera.getOrientation();
                    telemetry.addData("Position:", "%.2f, %.2f, %.2f", pos.x, pos.y, pos.z);
                    telemetry.addData("Yaw:", "%.2f", ori.getYaw(AngleUnit.DEGREES));
                    telemetry.addData("Pitch:", "%.2f", ori.getPitch(AngleUnit.DEGREES));
                    telemetry.addData("Roll:", "%.2f", ori.getRoll(AngleUnit.DEGREES));
                    // tagInCamera.getPosition() -> x, y, z (meters)
                    // tagInCamera.getOrientation() -> yaw, pitch, roll
                }

                // Field-relative robot pose (needs field map loaded)
                //Pnose3D botpose = result.getBotpose();
            }
        }

    }
}

