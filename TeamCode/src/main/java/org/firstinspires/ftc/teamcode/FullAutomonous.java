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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

@Autonomous(name = "FullAutomonous", group = "Autonomous")
public class FullAutomonous extends LinearOpMode {

    // ------------------- Pedro Pathing / Vision -------------------
    Pose currPose;
    Pose ballPose;
    Lodo lodo = new Lodo();
    double[] ballpos;

    // ------------------- Mechanisms (ported from TeleOp) -------------------
    private DcMotorEx intake_motor;
    private DcMotorEx control_motor;
    private DcMotorEx outtake_motor;

    // Intake: goBILDA 5203 19.2:1, 435 RPM target
    private static final double INTAKE_TICKS_PER_REV = 384.5;
    private static final double INTAKE_TARGET_RPM = 435;
    private static final double INTAKE_TICKS_PER_SEC =
            (INTAKE_TARGET_RPM * INTAKE_TICKS_PER_REV) / 60.0;

    // Control: goBILDA 5203 19.2:1
    private static final double CONTROL_TICKS_PER_REV = 384.5;
    private static final double CONTROL_RPM_STAGE_1 = 5;
    private static final double CONTROL_RPM_STAGE_2 = 1000;
    private static final double CONTROL_TICKS_PER_SEC_STAGE_1 =
            (CONTROL_RPM_STAGE_1 * CONTROL_TICKS_PER_REV) / 60.0;
    private static final double CONTROL_TICKS_PER_SEC_STAGE_2 =
            (CONTROL_RPM_STAGE_2 * CONTROL_TICKS_PER_REV) / 60.0;

    // Outtake: goBILDA 5203 Yellow Jacket (no gearbox, 6000 RPM), target 5500 RPM
    private static final double OUTTAKE_TICKS_PER_REV = 28.0;
    private static final double OUTTAKE_TARGET_RPM = 5500;
    private static final double OUTTAKE_TICKS_PER_SEC =
            (OUTTAKE_TARGET_RPM * OUTTAKE_TICKS_PER_REV) / 60.0;

    // ------------------- Automatic state machine -------------------
    // How close (cm — matches follower.getPose() now that Constants.java is cm-based) counts as "arrived at ball".
    // TUNE THIS to your field/robot geometry.
    private static final double ARRIVAL_RADIUS = 3.0;

    // How long control_motor spends at each stage during collection, in ms.
    // TUNE THESE to how long the 5 RPM "settle" stage and 1000 RPM "transfer" stage actually need.
    private static final long CONTROL_STAGE_1_DURATION_MS = 500;
    private static final long CONTROL_STAGE_2_DURATION_MS = 1000;
    private static final long COLLECT_DURATION_MS = CONTROL_STAGE_1_DURATION_MS + CONTROL_STAGE_2_DURATION_MS;

    private enum State {
        SEARCHING,   // no ball detected yet, everything idle
        APPROACHING, // ball detected, driving to it, intake running
        COLLECTING   // arrived at ball, running control transfer + outtake for a fixed duration
    }

    private State state = State.SEARCHING;
    private long collectStartTime = 0;

    @Override
    public void runOpMode() {
        // follower/telemetryM/poseHistory are only assigned inside Tuning.onSelect(),
        // which never runs when GoToBall is launched directly. Build everything here.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0)); // set this to your real starting pose
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Tuning.initPoseHistory(); // lets drawCurrentAndHistory() work from this package

        // ---------------- Hardware Map: Mechanisms ----------------
        intake_motor  = hardwareMap.get(DcMotorEx.class, "intake_motor");
        control_motor = hardwareMap.get(DcMotorEx.class, "control_motor");
        outtake_motor = hardwareMap.get(DcMotorEx.class, "outtake_motor");

        intake_motor.setDirection(DcMotor.Direction.FORWARD);
        control_motor.setDirection(DcMotor.Direction.FORWARD);
        outtake_motor.setDirection(DcMotor.Direction.REVERSE);

        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        control_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        control_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        control_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            }

            double distCm = distance(currPose, ballPose);

            // ============= STATE MACHINE =============
            switch (state) {

                case SEARCHING:
                    if (ballPose != null) {
                        // Found a ball: head toward it and start intaking.
                        PathChain toBall = follower.pathBuilder()
                                .addPath(new BezierLine(currPose, ballPose))
                                .setLinearHeadingInterpolation(currPose.getHeading(), ballPose.getHeading())
                                .build();
                        follower.followPath(toBall, true);
                        state = State.APPROACHING;
                    }
                    break;

                case APPROACHING:
                    // Re-plan toward the freshest ball position once the current path finishes,
                    // same behavior as before, but only while we're still far away.
                    if (distCm <= ARRIVAL_RADIUS) {
                        // Close enough: stop driving immediately, stop intake, start collect sequence.
                        follower.breakFollowing();
                        state = State.COLLECTING;
                        collectStartTime = System.currentTimeMillis();
                    } else if (ballPose != null && (follower.atParametricEnd() || !follower.isBusy())) {
                        PathChain triangle = follower.pathBuilder()
                                .addPath(new BezierLine(currPose, ballPose))
                                .setLinearHeadingInterpolation(currPose.getHeading(), ballPose.getHeading())
                                .build();
                        follower.followPath(triangle, true);
                    }
                    break;

                case COLLECTING:
                    if (System.currentTimeMillis() - collectStartTime >= COLLECT_DURATION_MS) {
                        // Done collecting this ball; go back to searching for the next one.
                        ballPose = null;
                        state = State.SEARCHING;
                    }
                    break;
            }

            // ============= MECHANISM OUTPUT (driven by state) =============
            boolean intakeOn   = (state == State.APPROACHING);
            intake_motor.setVelocity(intakeOn ? INTAKE_TICKS_PER_SEC : 0);

            if (state == State.COLLECTING) {
                long elapsed = System.currentTimeMillis() - collectStartTime;
                if (elapsed < CONTROL_STAGE_1_DURATION_MS) {
                    // Stage 1: slow settle, outtake not running yet.
                    control_motor.setVelocity(CONTROL_TICKS_PER_SEC_STAGE_1);
                    outtake_motor.setVelocity(0);
                } else {
                    // Stage 2: fast transfer, outtake spun up to score.
                    control_motor.setVelocity(CONTROL_TICKS_PER_SEC_STAGE_2);
                    outtake_motor.setVelocity(OUTTAKE_TICKS_PER_SEC);
                }
            } else {
                control_motor.setVelocity(0);
                outtake_motor.setVelocity(0);
            }

            telemetryM.addData("State", state);
            telemetryM.addData("Ball Pose", ballPose != null ? ballPose.toString() : "none");
            telemetryM.addData("Distance to Ball (cm)", distCm);
            telemetryM.addData("Arrival Radius (cm)", ARRIVAL_RADIUS);
            telemetryM.addData("Last ballpos length", ballpos != null ? ballpos.length : -1);
            telemetryM.addData("Intake vel (actual)", intake_motor.getVelocity());
            telemetryM.addData("Control vel (actual)", control_motor.getVelocity());
            telemetryM.addData("Outtake vel (actual)", outtake_motor.getVelocity());
            telemetryM.update();

            // Mirror to the standard Driver Station telemetry too, since Panels telemetry
            // only shows up on the Panels web dashboard, not the DS phone/tablet.
            telemetry.addData("State", state);
            telemetry.addData("Ball Pose", ballPose != null ? ballPose.toString() : "none");
            telemetry.addData("Distance to Ball (cm)", distCm);
            telemetry.addData("Arrival Radius (cm)", ARRIVAL_RADIUS);
            telemetry.addData("Last ballpos length", ballpos != null ? ballpos.length : -1);
            telemetry.addData("Intake vel (actual)", intake_motor.getVelocity());
            telemetry.addData("Control vel (actual)", control_motor.getVelocity());
            telemetry.addData("Outtake vel (actual)", outtake_motor.getVelocity());
            telemetry.update();
        }

        // Stop everything on exit
        intake_motor.setVelocity(0);
        control_motor.setVelocity(0);
        outtake_motor.setVelocity(0);
    }

    private double distance(Pose a, Pose b) {
        if (a == null || b == null) return Double.MAX_VALUE;
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}