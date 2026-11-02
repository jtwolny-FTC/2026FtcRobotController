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
public class RedPath1 extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Subsystems
    private ShootingSystem shootingSystem;
    private BallLoadingServo ballLoadingServo;
    private IntakeServo intakeSystem;
    private IntakeMotor intakeFeed;
    
    // Poses
    private final Pose startPose = poseFactory.of(82.9111, 133.5555, 270);
    private final Pose shootingPose = poseFactory.of(82.8036, 119.8316, 270);
    private final Pose gardenStart = poseFactory.of(82.8036, 119.8316, 270);
    private final Pose gardenPose = poseFactory.of(134.3023, 133.4371, 270);
    private final Pose gardenControl1 = poseFactory.of(134.2398, 97.3394, 0);
    private final Pose parkStart = poseFactory.of(134.3023, 133.4371, 270);
    private final Pose parkPose = poseFactory.of(132.3693, 44.1264, 0);
    private final Pose parkControl1 = poseFactory.of(99.5612, 78.5075, 0);

    // Path methods
    public Path shootingPath() {
        return Paths.line(startPose, shootingPose).linear(startPose, shootingPose);
    }

    public Path gardenPath() {
        return Paths.curve(gardenStart, gardenControl1, gardenPose).linear(gardenStart, gardenPose);
    }

    public Path parkPath() {
        return Paths.curve(parkStart, parkControl1, parkPose).linear(parkStart, parkPose);
    }
    
private Command autoRoutine() {
    return sequential(
            follow(follower, shootingPath()),
            instant(() -> shootingSystem.start()),
            waitMs(2000),
            instant(() -> ballLoadingServo.goAdvanceHalfTurn()),
            waitMs(1500),
            instant(() -> ballLoadingServo.goAdvanceHalfTurn()),
            waitMs(1500),
            instant(() -> shootingSystem.stop()),
            instant(() -> intakeSystem.start()),
            instant(() -> intakeFeed.start()),
            follow(follower, gardenPath()),
            waitMs(500),
            instant(() -> intakeSystem.stop()),
            instant(() -> intakeFeed.stop()),
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
