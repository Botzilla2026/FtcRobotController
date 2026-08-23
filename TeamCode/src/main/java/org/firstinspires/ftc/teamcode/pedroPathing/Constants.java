package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    // ---------------------------------------------------------------
    // FOLLOWER CONSTANTS
    // Units: cm (converted from an inches-based tune, factor 2.54).
    // mass is set from your robot weight. forwardZeroPowerAcceleration/
    // lateralZeroPowerAcceleration are physical decelerations and were
    // converted directly (in/sec^2 * 2.54 = cm/sec^2). The PIDF gains
    // below operate on cm-scale position/velocity error now instead of
    // inches — that changes their effective aggressiveness (errors are
    // numerically ~2.54x larger in cm), so they are NOT auto-converted.
    // RE-RUN the Automatic, PIDF, and centripetal tuners before trusting
    // these — headingPIDFCoefficients are in radians and are unaffected
    // by the unit change, but translational/drive PIDFs need retuning.
    // ---------------------------------------------------------------
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.34191) // kg — unaffected by length-unit change
            .forwardZeroPowerAcceleration(-71.12)  // -28.0 in/sec^2 * 2.54 — RUN THE TUNER to confirm
            .lateralZeroPowerAcceleration(-147.32) // -58.0 in/sec^2 * 2.54 — RUN THE TUNER to confirm
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.03, 0, 0, 0.015 // NOT yet retuned for cm — re-run translational tuner
            ))
            .translationalPIDFSwitch(10.16) // 4 in * 2.54 — distance threshold, converted directly
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.4, 0, 0.005, 0.0006 // NOT yet retuned for cm — re-run translational tuner
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.7, 0.008, 0.045, 0.01   // radians-based — unaffected by cm/in switch
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5, 0, 0.1, 0.0005       // radians-based — unaffected by cm/in switch
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1, 0, 0.00035, 0.6, 0.015 // NOT yet retuned for cm — re-run drive PIDF tuner
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02, 0, 0.000005, 0.6, 0.01 // NOT yet retuned for cm — re-run drive PIDF tuner
            ))
            .drivePIDFSwitch(38.1) // 15 in * 2.54 — distance threshold, converted directly
            .centripetalScaling(0.0005); // NOT yet retuned for cm

    // ---------------------------------------------------------------
    // MECANUM DRIVETRAIN CONSTANTS
    // Motor names/directions must match your hardware config exactly.
    // xVelocity/yVelocity converted from in/sec to cm/sec (factor 2.54).
    // Run the drive velocity tuner to replace these with real numbers.
    // ---------------------------------------------------------------
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("left_front_motor")
            .leftRearMotorName("left_back_motor")
            .rightFrontMotorName("right_front_motor")
            .rightRearMotorName("right_back_motor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(177.8)  // 70.0 in/sec * 2.54 — estimated from motor/wheel specs, retune
            .yVelocity(139.7); // 55.0 in/sec * 2.54 — estimated (mecanum strafe is slower), retune

    // ---------------------------------------------------------------
    // PINPOINT LOCALIZER CONSTANTS
    // Measured directly off your chassis, now in cm:
    //   forward pod: 23 cm left of center
    //   strafe pod:   7 cm behind center
    // Encoder resolution defaults to goBILDA_4_BAR_POD, which matches your
    // SKU 3110-0001-0002 (32mm wheel) pods — this is a fixed physical
    // encoder property, not affected by the cm/inches choice here.
    // Encoder directions below are placeholders — confirm/flip using the
    // localization test in the tuning OpMode (drive forward -> x should
    // increase; drive left -> y should increase).
    // ---------------------------------------------------------------
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("odoX")
            .forwardPodY(23.0)   // cm, left of center = positive
            .strafePodX(-7.0)    // cm, behind center = negative
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // ---------------------------------------------------------------
    // PATH CONSTRAINTS
    // Order: tValueConstraint, velocityConstraint, translationalConstraint,
    // headingConstraint, timeoutConstraint, brakingStrength,
    // BEZIER_CURVE_SEARCH_LIMIT (leave at 10), brakingStart
    // translationalConstraint is a length (how close counts as "reached"),
    // converted from 0.1 in to cm. headingConstraint is radians-based and
    // timeoutConstraint/brakingStrength/brakingStart are unitless — unaffected.
    // ---------------------------------------------------------------
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,  // tValueConstraint
            0.1,    // velocityConstraint (fraction of max speed — unitless)
            0.254,  // translationalConstraint: 0.1 in * 2.54 = cm
            0.009,  // headingConstraint (radians — unaffected)
            50,     // timeoutConstraint (ms)
            0.8,    // brakingStrength — lowered from 1.25, front-heavy bot was braking too hard, rear end sliding
            10,     // BEZIER_CURVE_SEARCH_LIMIT
            1       // brakingStart
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}