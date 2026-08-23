package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TestMoveForward10Right10", group = "Autonomous")
public class TestMoveForward10Right10 extends OpMode {

    private Follower follower;
    private Path forwardPath;
    private Path rightPath;
    private int pathState = 0;
    private long pauseStartTime = 0;

    private static final long PAUSE_MS = 5000; // 2 second pause

    // =========================================================================
    // INIT
    // =========================================================================
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        // X = forward, Y = right in Pedro's coordinate system
        forwardPath = new Path(new BezierLine(
                new Pose(0, 0, 0),
                new Pose(30, 0, 0)
        ));
        forwardPath.setConstantHeadingInterpolation(0);

        rightPath = new Path(new BezierLine(
                new Pose(30, 0, 0),
                new Pose(30, 30, 0)
        ));
        rightPath.setConstantHeadingInterpolation(0);

        telemetry.addData("Status", "Initialized — ready to run");
        telemetry.update();
    }

    // =========================================================================
    // START
    // =========================================================================
    @Override
    public void start() {
        follower.followPath(forwardPath, true);
        pathState = 1;
    }

    // =========================================================================
    // LOOP
    // =========================================================================
    @Override
    public void loop() {
        follower.update();

        switch (pathState) {

            case 1: // moving forward
                telemetry.addData("Time1 ", System.currentTimeMillis());
                telemetry.addData("X ", follower.getPose().getX());
                telemetry.addData("Y ", follower.getPose().getY());

                if (!follower.isBusy()) {
                    pauseStartTime = System.currentTimeMillis();
                    pathState = 2;

                }
                break;

            case 2: // pausing for 2 seconds
                follower.breakFollowing();
                telemetry.addData("Time2 ", System.currentTimeMillis());
                telemetry.addData("X2 ", follower.getPose().getX());
                telemetry.addData("Y2 ", follower.getPose().getY());

                if (System.currentTimeMillis() - pauseStartTime >= PAUSE_MS) {
                    follower.followPath(rightPath, true);
                    pathState = 3;

                }
                telemetry.addData("Time3 ", System.currentTimeMillis());
                telemetry.addData("X3 ", follower.getPose().getX());
                telemetry.addData("Y3 ", follower.getPose().getY());
                break;

            case 3: // moving right
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    pathState = 4;
                }
                break;

            case 4: // done
                follower.breakFollowing();
                break;
        }

        String stateLabel;
        switch (pathState) {
            case 1:  stateLabel = "Moving Forward"; break;
            case 2:  stateLabel = "Pausing";        break;
            case 3:  stateLabel = "Moving Right";   break;
            default: stateLabel = "Done";           break;
        }

        telemetry.addData("Path State",    stateLabel);
        telemetry.addData("X",             "%.2f", follower.getPose().getX());
        telemetry.addData("Y",             "%.2f", follower.getPose().getY());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

} // end class