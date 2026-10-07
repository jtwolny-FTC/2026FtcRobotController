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
public class SamplePath extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Start Point
    private final Pose startPose    = poseFactory.of(55, 8.5, 90);

    // Segment 1 (Curve points - control point assigned 0 heading)
    private final Pose path1Start   = poseFactory.of(23.641, 23.000, 90);
    private final Pose path1Control = poseFactory.of(17.793, 64.745, 0); // Heading placeholder (ignored by curve geometry)
    private final Pose path1End     = poseFactory.of(9.945, 104.490, 90);

    // Heading anchors for the curve's linear interpolation (44 to 90 degrees)
    private final Pose headingStart = poseFactory.of(0, 0, 44);
    private final Pose headingEnd   = poseFactory.of(0, 0, 90);

    // Segments 2 & 3 Target Poses
    private final Pose path2End     = poseFactory.of(53.927, 124.349, 90);
    private final Pose path3End     = poseFactory.of(117.568, 125.196, 90);


    // Path methods
// Connects your new start point to the beginning of your original path
    private Path initialConnect() {
        return line(startPose, path1Start).tangent();
    }

    // Segment 1: The original BezierCurve with linear heading interpolation
    private Path segment1Curve() {
        return curve(path1Start, path1Control, path1End).linear(headingStart, headingEnd);
    }

    // Segment 2: The first tangent BezierLine
    private Path segment2Line() {
        return line(path1End, path2End).tangent();
    }

// Segment 3: The second tangent BezierLine, driven in reverse
    private Path segment3Line() {
        return line(path2End, path3End)
                .reverseTangent();
    }


    private Command autoRoutine() {
        return sequential(
                follow(follower, initialConnect()),
                follow(follower, segment1Curve()),
                follow(follower, segment2Line()),
                follow(follower, segment3Line() ) // Driven in reverse
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
