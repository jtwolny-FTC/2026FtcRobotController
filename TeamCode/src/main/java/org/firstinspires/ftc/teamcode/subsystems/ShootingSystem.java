package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * ShootingSystem Subsystem for the robot.
 *
 * Controls a single shooter motor.
 */
public class ShootingSystem implements Subsystem {

    private static ShootingSystem INSTANCE = null;

    private Telemetry telemetry;
    
    // Single shooter motor
    private MotorEx shootingMotor = new MotorEx("sm1");

    // Tune these values for our robot
    private static final double DEFAULT_SHOOTING_VELOCITY = 1100.0;
    private static final double MIN_SHOOTING_VELOCITY = 800.0;
    private static final double MAX_SHOOTING_VELOCITY = 1800.0;
    private static final double SHOOTING_VELOCITY_INCREMENT = 50.0;

    /*
     * Velocity controller constants.
     *
     * kF provides the base power needed to maintain the requested velocity.
     * kP adds or removes power based on velocity error.
     *
     * These values are starting points and will need tuning.
     */
    private static final double VELOCITY_KF = 0.00045;
    private static final double VELOCITY_KP = 0.00020;

    /*
     * Percentage of target velocity required before the loader
     * is allowed to feed a ball.
     *
     * 0.90 means the measured velocity must be at least 90%
     * of the selected target velocity.
     */
    private static final double AT_SPEED_PERCENT = 0.90;

    /*
     * Prevents the shooter from being considered ready while
     * it is initially accelerating.
     */
    private static final long SPIN_UP_DELAY_MS = 500;

    private double targetShootingVelocity =
            DEFAULT_SHOOTING_VELOCITY;

    private boolean shooterAtSpeed = false;
    private boolean shooterRunning = false;

    private long shooterStartTime = 0;

    private ShootingSystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
   
    public static ShootingSystem getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new ShootingSystem(telemetry);
        }
        return INSTANCE;
    }
    
    public static ShootingSystem getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException(
                    "ShootingSystem must be initialized with telemetry first!");
        }
        return INSTANCE;
    }

    /**
     * Start the shooter using the currently selected target velocity.
     * This method can be called from Pedro IVY autonomous using:
     * instant(() -> shootingSystem.start())
     */
    public void start() {
        shooterRunning = true;
        shooterAtSpeed = false;
        shooterStartTime = System.currentTimeMillis();
    }

    /**
     * Stop the shooter.
     * This method can be called from Pedro IVY autonomous using:
     * instant(() -> shootingSystem.stop())
     */
    public void stop() {
        shooterRunning = false;
        shooterAtSpeed = false;

        shootingMotor.setPower(0.0);
    }
     /**
     * Stop the shooter, intake motor, intake servos,
     * and ball-loading servo.
     */
    public Command stopAllSubsystems = new InstantCommand(() -> {

        stop();
        // Stop intake and loader
        IntakeMotor.getInstance().stop();
        IntakeServo.getInstance().stop();
        BallLoadingServo.getInstance().stop();

    }).requires(this);

    // Returns true when the shooter has been commanded to run
    public boolean isRunning() {
    return shooterRunning;
    }

    public boolean isAtSpeed() {
        return shooterAtSpeed;
    }
    
    //Returns the current target shooter velocity
    public double getTargetShootingVelocity() {
        return targetShootingVelocity;
    }

    //Returns the current measured shooter velocity.
    public double getCurrentVelocity() {
        return Math.abs(shootingMotor.getVelocity());
    }

    /**
     * TeleOp command: Turn shooter on
     */
    public final Command on = 
	new InstantCommand(this::start)
                    .requires(this);

    /**
     * TeleOp command: Turn shooter Off
     */
    public final Command off =
            new InstantCommand(this::stop)
                    .requires(this);

    /**
     * Optional TeleOp command: toggle the shooter on or off.
     */
    public final Command startStop =
            new InstantCommand(() -> {
                if (shooterRunning) {
                    stop();
                } else {
                    start();
                }
            }).requires(this);

        /**
     * Increase target shooter velocity.
     *
     * If the shooter is already running, the new velocity
     * is sent to the motor immediately.
     */
    public final Command increaseShootingVelocity =
            new InstantCommand(() -> {
                targetShootingVelocity +=
                        SHOOTING_VELOCITY_INCREMENT;
                if (targetShootingVelocity >
                        MAX_SHOOTING_VELOCITY) {
                    targetShootingVelocity =
                            MAX_SHOOTING_VELOCITY;
                }
                shooterAtSpeed = false;
                if (shooterRunning) {
                    shooterStartTime =
                            System.currentTimeMillis();
                }
            }).requires(this);

    /**
     * Decrease target shooter velocity.
     *
     * If the shooter is already running, the new velocity
     * is sent to the motor immediately.
     */
    public final Command decreaseShootingVelocity =
            new InstantCommand(() -> {
                targetShootingVelocity -=
                        SHOOTING_VELOCITY_INCREMENT;
                if (targetShootingVelocity <
                        MIN_SHOOTING_VELOCITY) {
                    targetShootingVelocity =
                            MIN_SHOOTING_VELOCITY;
                }
                shooterAtSpeed = false;
                if (shooterRunning) {
                    shooterStartTime =
                            System.currentTimeMillis();
                }
            }).requires(this);

    /**
     * Limit a number to the specified range.
     */
    private double clamp(
            double value,
            double minimum,
            double maximum) {

        return Math.max(
                minimum,
                Math.min(maximum, value));
    }

    @Override
    public void periodic() {

        double currentVelocity =
                Math.abs(shootingMotor.getVelocity());

        if (shooterRunning) {         
            // Calculate velocity error.
            double velocityError =
                    targetShootingVelocity - currentVelocity;
            /*
             * Feedforward estimates the base power required
             * for the requested velocity.
             */
            double feedforwardPower =
                    targetShootingVelocity * VELOCITY_KF;
            /*
             * Proportional correction adds or removes power
             * based on the current velocity error.
             */
            double correctionPower =
                    velocityError * VELOCITY_KP;
            double requestedPower =
                    feedforwardPower + correctionPower;
            /*
             * This shooter only operates in one direction,
             * so power is limited from 0.0 to 1.0.
             */
            requestedPower =
                    clamp(requestedPower, 0.0, 1.0);
            shootingMotor.setPower(requestedPower);

        } else {
            shootingMotor.setPower(0.0);
        }

        double minimumReadyVelocity =
                targetShootingVelocity * AT_SPEED_PERCENT;
        boolean spinUpDelayComplete =
                System.currentTimeMillis() - shooterStartTime
                        >= SPIN_UP_DELAY_MS;        shooterAtSpeed =
                shooterRunning
                        && spinUpDelayComplete
                        && currentVelocity >= minimumReadyVelocity;

        telemetry.addData("<===== Shooting System =====>", "");
        telemetry.addData(
                "Shooter",
                "Pwr: %.2f | Tar: %.2f | Vel: %.2f",
                shootingMotor.getPower(),
                targetShootingVelocity,
                currentVelocity);
    }
}
