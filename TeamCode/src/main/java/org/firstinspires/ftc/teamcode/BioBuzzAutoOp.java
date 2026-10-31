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


@Autonomous(name = "BioBuzzAutoOp", group = "Autonomous")
public class BioBuzzAutoOp extends LinearOpMode{
    Pose currPose;
    Pose shootPose = new Pose(48.0, 0.0, 0.0);
    final double ARRIVAL_RADIUS = 1.0;
    double OUTTAKE_RPM = 3500;
    static final double OUTTAKE_TICKS_PER_REV = 28;   // goBILDA 5203 6000rpm motor (bare). Change if geared.
    double OUTTAKE_TICKS_PER_SEC = OUTTAKE_RPM * OUTTAKE_TICKS_PER_REV / 60.0;

    @Override
    public void runOpMode(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0)); // set this to your real starting pose
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.setMsTransmissionInterval(50);

        DcMotor leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");     // Control Hub motor port 0
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");    // Control Hub motor port 1
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");  // Control Hub motor port 2
        DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake"); // Control Hub motor port 3

        CRServo rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo"); // Expansion Hub servo port 0
        CRServo windmillServo    = hardwareMap.get(CRServo.class, "windmillServo");    // Expansion Hub servo port 1
        CRServo leftIntakeServo  = hardwareMap.get(CRServo.class, "leftIntakeServo");  // Expansion Hub servo port 2

        // ---------- Directions ----------
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);   // flip if robot drives backwards
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Side servos spin opposite directions to pull balls toward the center
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // ---------- Motor modes ----------
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            drawCurrentAndHistory();

            currPose = follower.getPose();

            double dx = shootPose.getX() - currPose.getX();
            double dy = shootPose.getY() - currPose.getY();

            double distance = Math.hypot(dx, dy);
            if (distance <= ARRIVAL_RADIUS) {

                // We've reached the ball
                follower.breakFollowing();
                telemetry.addData("ARRIVED", "true");
                telemetry.addLine("Shooting...");
                follower.update();

                outtakeMotor.setVelocity(OUTTAKE_TICKS_PER_SEC);



            } else if (!follower.isBusy()) {

                PathChain path = follower.pathBuilder()
                        .addPath(new BezierLine(currPose, shootPose))
                        .setLinearHeadingInterpolation(
                                currPose.getHeading(),
                                shootPose.getHeading()
                        )
                        .build();

                telemetry.addData("Following:", "true");
                follower.followPath(path, true);

            }
        }
    }
}
