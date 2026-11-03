package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.ChassisConstants.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
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
}
