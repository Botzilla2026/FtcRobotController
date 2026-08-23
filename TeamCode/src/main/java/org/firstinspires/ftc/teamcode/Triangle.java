package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Triangle", group = "Autonomous")
public class Triangle extends LinearOpMode {
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose interPose = new Pose(20, -20, Math.toRadians(0));
    private final Pose endPose = new Pose(20, 20, Math.toRadians(0));

    private PathChain triangle = follower.pathBuilder()
            .addPath(new BezierLine(startPose, interPose))
            .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
            .addPath(new BezierLine(interPose, endPose))
            .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
            .addPath(new BezierLine(endPose, startPose))
            .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
            .build();

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void runOpMode() {
        follower.update();
        drawCurrentAndHistory();

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }
    }
    /** Creates the PathChain for the "triangle".*/
}