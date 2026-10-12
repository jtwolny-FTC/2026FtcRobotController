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

    private ShootingSystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    
    public void stop() {
        shootingMotor.setPower(STOP_POWER);
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
    
    // Power levels for different shooting states
    private static final double LOW_THRESHOLD = 0.25;
    private static final double MID_POWER = 0.57;
    private static final double HIGH_POWER = 0.70;
    private static final double STOP_POWER = 0.0;

    private static final double SHOOTING_PWR_INCREMENT = 0.05;

    public Command stopAllSubsystems = new InstantCommand(() -> {

        shootingMotor.setPower(STOP_POWER);

        // Stop intake and loader
        IntakeServo.getInstance().stop();
        BallLoadingServo.getInstance().stop();

    }).requires(this);

    /**
     * Toggle shooter on/off.
     */
    public Command startStop = new InstantCommand(() -> {
        if (shootingMotor.getPower() <= 0.10) {
            shootingMotor.setPower(MID_POWER);
        } else {
            shootingMotor.setPower(STOP_POWER);
        }
    }).requires(this);

    /**
     * Toggle between medium and high power.
     */
    public Command toggleShootingPower = new InstantCommand(() -> {
        if (shootingMotor.getPower() <= 0.10
                || shootingMotor.getPower() >= HIGH_POWER) {

            shootingMotor.setPower(MID_POWER);

        } else {

            shootingMotor.setPower(HIGH_POWER);

        }
    }).requires(this);

    /**
     * Increase shooter power.
     */
    public Command increaseShootingPower = new InstantCommand(() -> {

        double newShootingPower =
                shootingMotor.getPower() + SHOOTING_PWR_INCREMENT;

        if (newShootingPower >= HIGH_POWER) {
            shootingMotor.setPower(HIGH_POWER);
        } else {
            shootingMotor.setPower(newShootingPower);
        }

    }).requires(this);

    /**
     * Decrease shooter power.
     */
    public Command decreaseShootingPower = new InstantCommand(() -> {

        double newShootingPower =
                shootingMotor.getPower() - SHOOTING_PWR_INCREMENT;

        if (newShootingPower <= LOW_THRESHOLD) {
            shootingMotor.setPower(LOW_THRESHOLD);
        } else {
            shootingMotor.setPower(newShootingPower);
        }

    }).requires(this);

    @Override
    public void periodic() {

        telemetry.addData("<===== Shooting System =====>", "");

        telemetry.addData(
                "Shooter",
                "Pwr: %.2f | Pos: %.2f | Vel: %.2f",
                shootingMotor.getPower(),
                shootingMotor.getCurrentPosition(),
                shootingMotor.getVelocity());

        telemetry.update();
    }
}
