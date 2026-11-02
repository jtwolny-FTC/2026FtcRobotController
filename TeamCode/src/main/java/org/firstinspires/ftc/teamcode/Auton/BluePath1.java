package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

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
public class BluePath1 extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose startPose = poseFactory.of(82.9111, 133.5555, 270);
    private final Pose shooting1 = poseFactory.of(82.8036, 119.8316, 270);
    private final Pose gardenStart = poseFactory.of(82.8036, 119.8316, 270);
    private final Pose garden = poseFactory.of(134.3023, 133.4371, 270);
    private final Pose gardenControl1 = poseFactory.of(134.2398, 97.3394, 0);
    private final Pose parkStart = poseFactory.of(134.3023, 133.4371, 270);
    private final Pose park = poseFactory.of(132.3693, 44.1264, 0);
    private final Pose parkControl1 = poseFactory.of(99.5612, 78.5075, 0);

    // Path methods
    public Path shooting1() {
        return Paths.line(startPose, shooting1).linear(start, shooting1);
    }

    public Path garden() {
        return Paths.curve(gardenStart, gardenControl1, garden).linear(gardenStart, garden);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
    
    private Command autoRoutine() {
        return sequential(
                follow(follower, shooting1()),
                follow(follower, garden()),
                follow(follower, park())
               );
    }

    @Override
    public void init() {
        Scheduler.reset();

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
