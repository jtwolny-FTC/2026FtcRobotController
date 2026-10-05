package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.ChassisConstants.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ChassisConstants;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.44)
            .forwardZeroPowerAcceleration(-31.0151)
            .lateralZeroPowerAcceleration(-53.9751)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.01, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.02, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.007, 0, 0.0002, 0.2, 0.0005));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName(LEFT_FRONT_MOTOR_NAME)
            .leftRearMotorName(LEFT_REAR_MOTOR_NAME)
            .rightFrontMotorName(RIGHT_FRONT_MOTOR_NAME)
            .rightRearMotorName(RIGHT_REAR_MOTOR_NAME)
            .leftFrontMotorDirection(LEFT_FRONT_MOTOR_DIRECTION)
            .leftRearMotorDirection(LEFT_REAR_MOTOR_DIRECTION)
            .rightFrontMotorDirection(RIGHT_FRONT_MOTOR_DIRECTION)
            .rightRearMotorDirection(RIGHT_REAR_MOTOR_DIRECTION)
            .xVelocity(61.3148)
            .yVelocity(51.9894);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardPodY(-.5)
            .strafePodX(.5)
            .forwardEncoder_HardwareMapName(RIGHT_FRONT_MOTOR_NAME)
            .strafeEncoder_HardwareMapName(LEFT_FRONT_MOTOR_NAME)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.00294)
            .strafeTicksToInches(0.00293)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}

