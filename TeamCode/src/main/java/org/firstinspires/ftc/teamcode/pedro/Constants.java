package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.3);
        Controller secondaryTranslationalForward = Controller.proportional(0.1);
        Controller primaryTranslationalLateral = Controller.proportional(0.3);
        Controller secondaryTranslationalLateral = Controller.proportional(0.1);

        c.forwardTranslational.set(
                Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward)
        );
        c.strafeTranslational.set(
                Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral)
        );
        c.headingFeedback.set(Controller.proportional(5.25));
        c.coast.set(Controller.proportionalFeedforward(0.0109));
        c.brake.set(Controller.proportionalFeedforward(0.0087));
        c.maxAchievableForwardVelocity.set(72.7);
        c.maxAchievableStrafeVelocity.set(52.3);
        c.naturalForwardDeceleration.set(85.0);
        c.naturalStrafeDeceleration.set(104.5);
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0564, 0.0063));
        c.linearBrakeCoefficients.set(Matrix.diag(0.1060, 0.0871));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014, 0.0013));
    });

    public static MecanumConfig driveConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left");
        c.frontRightName.set("front_right");
        c.backLeftName.set("back_left");
        c.backRightName.set("back_right");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig pinpointConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, pinpointConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig));
    }
}