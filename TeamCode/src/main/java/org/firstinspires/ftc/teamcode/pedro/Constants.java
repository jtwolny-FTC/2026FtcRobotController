package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.ChassisConstants.*;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathTracker;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Collections;
import java.util.Map;

public class Constants {

    public static MecanumConfig driveConfig = new MecanumConfig(
    c -> {
        c.frontLeftName.set(LEFT_FRONT_MOTOR_NAME);
        c.backLeftName.set(LEFT_REAR_MOTOR_NAME);
        c.frontRightName.set(RIGHT_FRONT_MOTOR_NAME);
        c.backRightName.set(RIGHT_REAR_MOTOR_NAME);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    }
);
    public static Follower create(HardwareMap h) {
        return new Follower(
                new TwoWheelLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }

    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("driveWheel");
        c.yPodName.set("strafeWheel");
        c.imuName.set("imu");
        c.xPodOffset.set(0.23790568514519977);
        c.yPodOffset.set(0.8132518274454786);
        c.forwardTicksToInches.set(0.001974786185329457);
        c.strafeTicksToInches.set(0.5936657590860075);
        c.xPodDirection.set(Encoder.REVERSE);
        c.yPodDirection.set(Encoder.FORWARD);
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
    });
        public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.21673573246597627);
                Controller secondaryTranslationalForward = Controller.proportional(0.0800780618607471);
                Controller primaryTranslationalLateral = Controller.proportional(0.9422513238995138);
                Controller secondaryTranslationalLateral = Controller.proportional(0.3481366867617966);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018666030678729042));
                c.brake.set(Controller.proportionalFeedforward(0.015866126076919684));

                c.headingFeedback.set(Controller.proportional(3.4459241569791197));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04333768992120032, 0.010806452497216587));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06199817893378212, 0.050418129135617856));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017202583013053256, 0.0020499811297467444));

                c.maxAchievableForwardVelocity.set(54.110783579053916);
                c.maxAchievableStrafeVelocity.set(42.44155458197613);
                c.naturalForwardDeceleration.set(38.861658978051345);
                c.naturalStrafeDeceleration.set(58.747237475924365);
            }
    );
}
