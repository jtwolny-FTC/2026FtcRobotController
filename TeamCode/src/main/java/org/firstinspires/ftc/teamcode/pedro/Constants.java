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
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(-5.517377477931226);
        c.yPodOffset.set(-0.7498509865107499);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
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

                // Higher value brakes less, lower value brakes more    
                c.brakeAggression.set(0.6);
                
                // how much power is used to stop the robot
                // Higher the value is quicker stopping
                c.maxBrakingPower.set(0.2);

                c.maxVelocityConstraint.set(20.0); // inches/second
                c.maxAccelerationConstraint.set(20.0); // inches/second^2
                c.maxDecelerationConstraint.set(50.0); // inches/second^2
                // sets speed the robot has to coast down to be at zero
                c.coastDownToVelocity.set(0.0);
                
                /** defines how close the robot's progress along the Bezier curve
                *  must be to 1 before the path segment is considered parametrically complete
                * Increasing this value allows the path to reach its parametric end earlier
                */
                c.parametricTConstraint.set(0.025);

                /** sets the velocity threshold used while correcting at the final Pose. 
                * This prevents the path from completing when the robot is still moving fast.
                * The robot's velocity must be lower than the threshold for the velocityConstraint to be met
                */    
                c.velocityConstraint.set(0.1);
                // sets the maximum allowed distance between the robot's position and the final target position
                c.translationalConstraint.set(0.1);

                // sets the maximum allowed heading error between the robot's heading and the final target heading
                c.headingConstraint.set(Math.toRadians(0.4));

                //s ets how long the follower is allowed to correct at the final Pose before the timeout constraint is met
                c.timeoutConstraint.set(100.0);

            }
    );
}
