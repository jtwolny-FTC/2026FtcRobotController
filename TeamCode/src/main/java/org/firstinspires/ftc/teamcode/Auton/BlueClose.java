package org.firstinspires.ftc.teamcode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueClose", group = "Autonomous")
@Configurable
public class BlueClose extends LinearOpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    // Motors
    private DcMotor FLM, RLM, FRM, RRM, Intake, Push;
    private DcMotorEx Shoot;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22, 121, Math.toRadians(140)));

        // Path generation
        paths = new Paths(follower);

        // Hardware
        FLM = hardwareMap.dcMotor.get("FLM");
        RLM = hardwareMap.dcMotor.get("RLM");
        FRM = hardwareMap.dcMotor.get("FRM");
        RRM = hardwareMap.dcMotor.get("RRM");
        Intake = hardwareMap.dcMotor.get("intake");
        Push = hardwareMap.dcMotor.get("push");

        FLM.setDirection(DcMotor.Direction.REVERSE);
        RLM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.FORWARD);
        RRM.setDirection(DcMotor.Direction.FORWARD);
        Push.setDirection(DcMotor.Direction.REVERSE);
        Shoot = hardwareMap.get(DcMotorEx.class, "shoot");
        Shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        imu.resetYaw();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        waitForStart();

        if (isStopRequested()) return;

        Shoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(7.5, 5.0, 0.0, 0.0));

        // -------------------------
        // AUTONOMOUS SEQUENCE
        // -------------------------

        Shoot.setVelocity((700 / 60) * 112);
        // PRELOAD
        follower.followPath(paths.scorePreLoad);
        runFollower();

        sleep(1000);
        ShootCycle();

        //sleep(2000);

        follower.followPath(paths.turnPickup1);
        runFollower();

        sleep(2000);
        stopAll();
        Intake.setPower(1);

        /*
        follower.followPath(paths.grabPickup1);
        runFollower();
        */

        runtime.reset();
        while (runtime.seconds() < 2.6) {
            FLM.setPower(-0.2);
            RLM.setPower(-0.2);
            FRM.setPower(-0.2);
            RRM.setPower(-0.2);
        }

        runtime.reset();
        while (runtime.seconds() < 1) {
            Shoot.setVelocity((-700 / 60) * 112);
            Push.setPower(-0.1);
        }

        Shoot.setVelocity((700 / 60) * 112);

        sleep(500);
        follower.followPath(paths.scorePickup1);
        runFollower();
        ShootCycle();

        follower.followPath(paths.grabPickup2);
        runFollower();

        stopAll();
    }

    private void ShootCycle() {
        if (isStopRequested()) return;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.75) {
            Shoot.setVelocity((700 / 60) * 112);
            Intake.setPower(1);
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            Shoot.setVelocity((700 / 60) * 112);
            Push.setPower(0.8);
            Intake.setPower(1);
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {
            Shoot.setVelocity((700 / 60) * 112);
            Push.setPower(-1);
            Intake.setPower(0);
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            Shoot.setVelocity((700 / 60) * 112);
            Push.setPower(0.8);
            Intake.setPower(1);
        }

        stopAll();
    }

    // -----------------------------------------------------
    // FOLLOWER RUNNER
    // -----------------------------------------------------
    private void runFollower() {
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }
    }

    private void stopAll() {
        Shoot.setPower(0);
        Push.setPower(0);
        Intake.setPower(0);
    }

    // -----------------------------------------------------
    // PATH DEFINITIONS
    // -----------------------------------------------------

    public static class Paths {
        public PathChain scorePreLoad, turnPickup1, grabPickup1, scorePickup1;
        public PathChain grabPickup2 , finishPickup2, scorePickup2;

        public Paths(Follower follower) {
            scorePreLoad = follower.pathBuilder()
                    .addPath(new Paths.line(new Pose(22, 121), new Pose(48, 96)))
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(135))
                    .build();

            turnPickup1 = follower.pathBuilder()
                    .addPath(new Paths.line(new Pose(48, 96), new Pose(45, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(5))
                    .build();

            grabPickup1 = follower.pathBuilder()
                    .addPath(new Paths.line(new Pose(84, 84), new Pose(127, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(5), Math.toRadians(5))
                    .setReversed()
                    .build();

            scorePickup1 = follower.pathBuilder()
                    .addPath(new Paths.line(new Pose(17, 84), new Pose(48, 96)))
                    .setLinearHeadingInterpolation(Math.toRadians(5), Math.toRadians(135))
                    .build();

            grabPickup2 = follower.pathBuilder()
                    .addPath(new Paths.line(new Pose(48, 96), new Pose(32, 71)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            finishPickup2 = follower.pathBuilder()
                    .addPath(new Paths.line(new Pose(102, 58), new Pose(134, 58)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            scorePickup2 = follower.pathBuilder()
                    .addPath(new Paths.line(new Pose(134, 58), new Pose(84, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();
        }
    }
}
