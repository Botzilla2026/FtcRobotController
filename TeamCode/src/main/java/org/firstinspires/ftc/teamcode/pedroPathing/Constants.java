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
    // mass is set from your robot weight. Everything else below is a
    // reasonable *starting* value from a typical mecanum FTC bot —
    // run the Automatic, PIDF, and centripetal tuners to correct them.
    // ---------------------------------------------------------------
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.34191) // kg — your actual robot weight
            .forwardZeroPowerAcceleration(-28.0)   // starter guess — RUN THE TUNER, this can't be derived from specs
            .lateralZeroPowerAcceleration(-58.0)   // starter guess — RUN THE TUNER, this can't be derived from specs
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.03, 0, 0, 0.015
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.4, 0, 0.005, 0.0006
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.7, 0.008, 0.045, 0.01   // reverted P back to 1.5 — confirmed better than 1.1
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5, 0, 0.1, 0.0005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1, 0, 0.00035, 0.6, 0.015
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02, 0, 0.000005, 0.6, 0.01
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    // ---------------------------------------------------------------
    // MECANUM DRIVETRAIN CONSTANTS
    // Motor names/directions must match your hardware config exactly.
    // xVelocity/yVelocity estimated from goBILDA 5203 (435 RPM), 104mm
    // wheels, direct drive (1:1). Theoretical max ~93 in/sec; real-world
    // achievable is ~75-80% of that with battery sag and mecanum losses.
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
            .xVelocity(70.0)   // in/sec — estimated from motor/wheel specs, retune
            .yVelocity(55.0);  // in/sec — estimated (mecanum strafe is slower), retune

    // ---------------------------------------------------------------
    // PINPOINT LOCALIZER CONSTANTS
    // Measured directly off your chassis:
    //   forward pod: 23 cm left of center   -> +9.06 in (left = positive)
    //   strafe pod:   7 cm behind center    -> -2.76 in (behind = negative)
    // Encoder resolution defaults to goBILDA_4_BAR_POD, which matches your
    // SKU 3110-0001-0002 (32mm wheel) pods, so no custom resolution needed.
    // Encoder directions below are placeholders — confirm/flip using the
    // localization test in the tuning OpMode (drive forward -> x should
    // increase; drive left -> y should increase).
    // ---------------------------------------------------------------
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("odoX")
            .forwardPodY(9.06)    // inches, left of center = positive
            .strafePodX(-2.76)    // inches, behind center = negative
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // ---------------------------------------------------------------
    // PATH CONSTRAINTS
    // Order: tValueConstraint, velocityConstraint, translationalConstraint,
    // headingConstraint, timeoutConstraint, brakingStrength,
    // BEZIER_CURVE_SEARCH_LIMIT (leave at 10), brakingStart
    // ---------------------------------------------------------------
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,  // tValueConstraint
            0.1,    // velocityConstraint
            0.1,    // translationalConstraint
            0.009,  // headingConstraint
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