package org.firstinspires.ftc.teamcode.Autonomous.OdemetryController;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.OdemetryController.PIDController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * OdometryMecanumDrive
 *
 * Integrates two odometry wheels + IMU heading into a global X/Y/heading pose and drives
 * a mecanum drivetrain using three PID loops (X, Y, heading).
 *
 * Uses the same odometry scaling and encoder device names as your SimplifiedOdometryRobot:
 *   - drive encoder device name: "shootL"  (axial/front-back)
 *   - strafe encoder device name: "push"  (lateral/left-right)
 *   - IMU device name: "imu"
 *
 * Wheel motor names: "FLM", "FRM", "RLM", "RRM"
 *
 * Your offsets: strafeOffset = 5.5 inches (positive left), forwardOffset = 0.0 inches
 */
public class OdometryMecanumDrive {
    // === Odometry constants (copied from provided odom class) ===
    private final double ODOM_INCHES_PER_COUNT   = 0.0049;   // GoBilda Odometry Pod (1/226.8)
    private final boolean INVERT_DRIVE_ODOMETRY  = true;     // when driving FORWARD odom count must increase
    private final boolean INVERT_STRAFE_ODOMETRY = true;     // when strafing LEFT odom must increase

    // your supplied offsets:
    private final double FORWARD_POD_OFFSET_IN = 0.0;    // forward/back pod offset from robot center (inches)
    private final double STRAFE_POD_OFFSET_IN  = 5.5;    // lateral pod offset from robot center (inches). +ve means left of center

    // Motor & sensor hardware
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private DcMotor driveEncoder;   // axial encoder (may be motor or odometry pod)
    private DcMotor strafeEncoder;  // lateral encoder

    private IMU imu;
    private HardwareMap hardwareMap;

    // Global pose (in inches / radians)
    private double globalX = 0.0;   // inches
    private double globalY = 0.0;   // inches
    private double headingRadians = 0.0; // radians, world frame

    // Internal odometry state
    private int lastDriveCounts = 0;
    private int lastStrafeCounts = 0;
    private double lastImuHeadingDeg = 0.0;

    // convenience (converted distances from odom)
    private double rawDriveDistance = 0.0;   // inches reported from encoder scale (absolute - offset)
    private double rawStrafeDistance = 0.0;  // inches

    // PID controllers (you can tune these)
    private PIDController pidX;   // strafe / lateral (global X)
    private PIDController pidY;   // forward / axial (global Y)
    private PIDController pidTheta; // heading (radians)

    // timing
    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0.0;

    // telemetry toggle (if you pass telemetry in opmode, you can print manually)
    private boolean showTelemetry = false;

    public OdometryMecanumDrive() {
        // default PID values — tune for your robot
        double dt = 0.02;
        pidX = new PIDController(0.8, 0.0, 0.0, dt, 0.0);        // simple P on X by default
        pidY = new PIDController(0.8, 0.0, 0.0, dt, 0.0);        // simple P on Y by default
        pidTheta = new PIDController(6.0, 0.0, 0.0, dt, 0.0);    // P on heading (radians)
    }

    /**
     * Initialize hardware. Call from your OpMode's init/start area.
     */
    public void initialize(HardwareMap hw, boolean showTelemetry) {
        this.hardwareMap = hw;
        this.showTelemetry = showTelemetry;

        leftFrontDrive  = hw.get(DcMotor.class, "FLM");
        rightFrontDrive = hw.get(DcMotor.class, "FRM");
        leftBackDrive   = hw.get(DcMotor.class, "RLM");
        rightBackDrive  = hw.get(DcMotor.class, "RRM");

        // Set directions so positive power makes robot go forward
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveEncoder = hw.get(DcMotor.class, "push");
        strafeEncoder = hw.get(DcMotor.class, "intake");

        imu = hw.get(IMU.class, "imu");

        // Reset encoders so odometry starts at zero reference
        driveEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // read initial values
        lastDriveCounts = driveEncoder.getCurrentPosition() * (INVERT_DRIVE_ODOMETRY ? -1 : 1);
        lastStrafeCounts = strafeEncoder.getCurrentPosition() * (INVERT_STRAFE_ODOMETRY ? -1 : 1);

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        lastImuHeadingDeg = orientation.getYaw(AngleUnit.DEGREES);

        globalX = 0;
        globalY = 0;
        headingRadians = Math.toRadians(lastImuHeadingDeg);

        timer.reset();
        lastTime = timer.seconds();
    }

    /**
     * Call repeatedly to update odometry from encoders + IMU.
     * This implements the standard two-wheel odometry integration:
     *   - Get Δforward and Δstrafe in robot frame (inches)
     *   - Rotate to world frame using current heading and add to globalX/globalY
     *
     * Note: heading comes from IMU (degrees).
     */
    public void updateOdometry() {
        // Read raw encoder counts (apply invert flags)
        int rawDrive = driveEncoder.getCurrentPosition() * (INVERT_DRIVE_ODOMETRY ? -1 : 1);
        int rawStrafe = strafeEncoder.getCurrentPosition() * (INVERT_STRAFE_ODOMETRY ? -1 : 1);

        // Convert counts to inches (absolute)
        double driveInches = (rawDrive) * ODOM_INCHES_PER_COUNT;
        double strafeInches = (rawStrafe) * ODOM_INCHES_PER_COUNT;

        // Compute deltas since last read (robot-frame distances)
        double deltaDrive = driveInches - (lastDriveCounts * ODOM_INCHES_PER_COUNT); // in inches
        double deltaStrafe = strafeInches - (lastStrafeCounts * ODOM_INCHES_PER_COUNT);

        // Update stored last count snapshots
        lastDriveCounts = rawDrive;
        lastStrafeCounts = rawStrafe;

        // get IMU heading (absolute), compute radians
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentHeadingDeg = orientation.getYaw(AngleUnit.DEGREES);
        // normalize heading difference using IMU absolute angle (we treat heading as continuous)
        headingRadians = Math.toRadians(currentHeadingDeg);

        // Robot frame: forward = +drive, left = +strafe
        // Transform robot-frame deltas to world-frame and integrate to global pose
        // worldX += Δdrive * cos(theta) - Δstrafe * sin(theta)
        // worldY += Δdrive * sin(theta) + Δstrafe * cos(theta)
        double cosH = Math.cos(headingRadians);
        double sinH = Math.sin(headingRadians);

        double dGlobalX = deltaDrive * cosH - deltaStrafe * sinH;
        double dGlobalY = deltaDrive * sinH + deltaStrafe * cosH;

        globalX += dGlobalX;
        globalY += dGlobalY;

        // Save last reported distances too (for external access if needed)
        rawDriveDistance = driveInches;
        rawStrafeDistance = strafeInches;
    }

    /**
     * High level drive-to target function (blocking). Call from LinearOpMode.
     * - targetX, targetY are in inches in the same world frame used by odometry
     * - targetHeadingDeg is desired absolute heading in degrees (CCW positive)
     *
     * This is a simple PID-based controller that runs until the robot is within tolerances.
     */
    public void driveTo(double targetX, double targetY, double targetHeadingDeg, double maxPower, double positionToleranceInches, double headingToleranceDeg, double hold, com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) {
        // set PID setpoints
        pidX.setSetpoint(targetX);
        pidY.setSetpoint(targetY);
        pidTheta.setSetpoint(Math.toRadians(targetHeadingDeg));

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // main control loop
        while (opMode.opModeIsActive()) {
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 0.02;
            lastTime = now;

            updateOdometry();

            // compute errors in world coordinates
            double errorX = targetX - globalX;
            double errorY = targetY - globalY;
            double errorTheta = normalizeAngleRadians(Math.toRadians(targetHeadingDeg) - headingRadians);

            // Stop condition check
            boolean posOk = (Math.hypot(errorX, errorY) <= positionToleranceInches);
            boolean headingOk = (Math.abs(Math.toDegrees(errorTheta)) <= headingToleranceDeg);

            if (posOk && headingOk) {
                if (holdTimer.seconds() > hold) { // hold 200ms
                    break;
                }
            } else {
                holdTimer.reset();
            }

            // PID outputs (we'll clamp to maxPower)
            // Note: our PIDController expects measured value; we set setpoints to target and call step(measured)
            double outX = pidX.step(globalX); // strafe command (left positive)
            double outY = pidY.step(globalY); // forward command (forward positive)
            double outTheta = pidTheta.step(headingRadians); // rotational command (radians -> convert to power)

            // convert rotation command (rad output) into a motor-power-like value.
            // The pidTheta tuning should be chosen so outTheta is roughly in [-maxPower, maxPower]
            // Clamp each axis
            outX = clamp(outX, -maxPower, maxPower);
            outY = clamp(outY, -maxPower, maxPower);
            outTheta = clamp(outTheta, -maxPower, maxPower);

            // move robot with those axis commands
            moveRobot(outY, outX, outTheta);

            // optionally show telemetry by opMode.telemetry if desired
            if (showTelemetry) {
                opMode.telemetry.addData("Pose", "X=%.2f Y=%.2f H=%.1f", globalX, globalY, Math.toDegrees(headingRadians));
                opMode.telemetry.addData("Err", "ex=%.2f ey=%.2f eth=%.1f", errorX, errorY, Math.toDegrees(errorTheta));
                opMode.telemetry.addData("Cmd", "fwd=%.2f str=%.2f yaw=%.2f", outY, outX, outTheta);
                opMode.telemetry.update();
            }

            opMode.sleep(10);
        }

        // ensure motors are stopped at end
        stopRobot();
    }

    /**
     * Low-level mecanum mixer. drive=forward (+), strafe=left (+), yaw=CCW (+)
     */
    public void moveRobot(double drive, double strafe, double yaw) {
        double lF = drive - strafe - yaw;
        double rF = drive + strafe + yaw;
        double lB = drive + strafe - yaw;
        double rB = drive - strafe + yaw;

        // normalize if any magnitude > 1.0
        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));
        if (max > 1.0) {
            lF /= max;
            rF /= max;
            lB /= max;
            rB /= max;
        }

        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);
    }

    public void stopRobot() {
        moveRobot(0,0,0);
    }

    // === Getters ===
    public double getGlobalX() { return globalX; }
    public double getGlobalY() { return globalY; }
    public double getHeadingRadians() { return headingRadians; }
    public double getHeadingDegrees() { return Math.toDegrees(headingRadians); }

    // === Utility ===
    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double normalizeAngleRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0*Math.PI;
        while (angle <= -Math.PI) angle += 2.0*Math.PI;
        return angle;
    }

    // allow tuning PIDs externally
    public void setPidXTunings(double kp, double ki, double kd, double dt) {
        pidX.setTunings(kp, ki, kd);
        // pid class stores dt in constructor; for simplicity we won't change dt here.
    }
    public void setPidYTunings(double kp, double ki, double kd, double dt) {
        pidY.setTunings(kp, ki, kd);
    }
    public void setPidThetaTunings(double kp, double ki, double kd, double dt) {
        pidTheta.setTunings(kp, ki, kd);
    }
}
