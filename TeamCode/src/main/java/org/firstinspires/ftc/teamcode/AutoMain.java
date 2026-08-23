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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

@Autonomous(name = "AutoMain", group = "Autonomous")
public class AutoMain extends LinearOpMode {
    Pose currPose;
    Pose ballPose;
    Lodo lodo = new Lodo();
    IntakeAuto intaker = new IntakeAuto(hardwareMap);
    double[] ballpos;

    @Override
    public void runOpMode() {
        // follower/telemetryM/poseHistory are only assigned inside Tuning.onSelect(),
        // which never runs when GoToBall is launched directly. Build everything here.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0)); // set this to your real starting pose
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.setMsTransmissionInterval(50);

        Tuning.initPoseHistory(); // lets drawCurrentAndHistory() work from this package

        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            drawCurrentAndHistory();

            currPose = follower.getPose();

            ballpos = lodo.getBall(
                    new double[]{currPose.getX(), currPose.getY(), Math.toDegrees(currPose.getHeading())},
                    hardwareMap
            );

            if (ballpos != null && ballpos.length != 0) {
                ballPose = new Pose(ballpos[0], ballpos[1], Math.toRadians(ballpos[2]));
                telemetry.addData("Ball Position:", "(x,y): (%.2f,%.2f)",ballpos[0], ballpos[1]);
                telemetry.update();
                if (follower.atParametricEnd() || !follower.isBusy()) {
                    PathChain triangle = follower.pathBuilder()
                            .addPath(new BezierLine(currPose, ballPose))
                            .setLinearHeadingInterpolation(currPose.getHeading(), ballPose.getHeading())
                            .build();

                    intaker.takein(435);
                    follower.followPath(triangle, true);
                    intaker.stoptake();
                }
            }

            telemetryM.update();
        }
    }
}