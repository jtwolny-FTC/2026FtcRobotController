package org.firstinspires.ftc.teamcode.Auton;

import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;
import org.firstinspires.ftc.teamcode.subsystems.BallLoadingServo;
import org.firstinspires.ftc.teamcode.subsystems.IntakeServo;
import org.firstinspires.ftc.teamcode.subsystems.IntakeMotor;

import com.pedropathing.api.Paths;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class RedPath2 extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Subsystems
    private ShootingSystem shootingSystem;
    private BallLoadingServo ballLoadingServo;
    private IntakeServo intakeSystem;
    private IntakeMotor intakeFeed;
    
    // Poses
    private final Pose startPose = poseFactory.of(82.75, 7.75, 90);
    private final Pose shootingPose = poseFactory.of(82.75, 28.75, 90);
    private final Pose parkStart = poseFactory.of(82.75, 28.75, 90);
    private final Pose parkPose = poseFactory.of(133.75, 25.75, 0);
    private final Pose parkControl1 = poseFactory.of(91.75, 19.75, 0);

    // Path methods
    public Path shootingPath() {
        return Paths.line(startPose, shootingPose).linear(startPose, shootingPose);
    }

    public Path parkPath() {
        return Paths.curve(parkStart, parkControl1, parkPose).tangent();
    }
    
private Command autoRoutine() {
    return sequential(
            follow(follower, shootingPath()),
            waitMs(10000),
            instant(() -> shootingSystem.start()),
            waitMs(2000),
            instant(() -> ballLoadingServo.funForward()),
            waitMs(5000),
            instant(() -> ballLoadingServo.stop()),
            instant(() -> shootingSystem.stop()),
            follow(follower, parkPath())       
    );
}

    @Override
    public void init() {
        Scheduler.reset();

        shootingSystem = ShootingSystem.getInstance(telemetry);
        ballLoadingServo = BallLoadingServo.getInstance(telemetry);
        intakeSystem = IntakeServo.getInstance(telemetry);
        intakeFeed = IntakeMotor.getInstance(telemetry);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    // from the example code at the end of the autonomous page
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        // add your other methods needed in the loop here
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());

        // We will add these two, but you can use the many other follower methods aswell in your own code!
        telemetry.addData("Path completion", follower.completion());
        telemetry.addData("Distance remaining in path", follower.remainingDistance());
        telemetry.update();
    }
}
