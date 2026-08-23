package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "MoveAtoB", group = "Autonomous")
public class MoveAtoB extends OpMode {

    private Follower follower;
    private Path path;
    private boolean pathStarted = false;

    private static final Pose START = new Pose(0, 0, 0);
    private static final Pose END   = new Pose(24, 0, 0);

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);

        path = new Path(new BezierLine(START, END));
        path.setConstantHeadingInterpolation(0);

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(path, true);
        pathStarted = true;
    }

    @Override
    public void loop() {
        follower.update();

        // Constantly track and print X and Y every loop
        double currentX = follower.getPose().getX();
        double currentY = follower.getPose().getY();
        double currentHeading = Math.toDegrees(follower.getPose().getHeading());

        telemetry.addLine("--- LIVE POSITION TRACKING ---");
        telemetry.addData("Current X (forward)", "%.4f inches", currentX);
        telemetry.addData("Current Y (right)",   "%.4f inches", currentY);
        telemetry.addData("Heading",              "%.4f degrees", currentHeading);
        telemetry.addLine("------------------------------");
        telemetry.addData("Target X", "%.4f inches", END.getX());
        telemetry.addData("Target Y", "%.4f inches", END.getY());
        telemetry.addData("Error X",  "%.4f inches", END.getX() - currentX);
        telemetry.addData("Error Y",  "%.4f inches", END.getY() - currentY);
        telemetry.addLine("------------------------------");

        if (pathStarted && !follower.isBusy()) {
            telemetry.addData("Status", "DONE — arrived at target");
        } else {
            telemetry.addData("Status", "MOVING — tracking position...");
        }

        telemetry.update();
    }
}