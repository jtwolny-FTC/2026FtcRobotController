package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeMotor implements Subsystem {

    private static IntakeMotor INSTANCE = null;

    private final Telemetry telemetry;

    // Motor name from Robot Configuration
    private final MotorEx intakeMotor = new MotorEx("intakeMotor");

    private static final double INTAKE_POWER = 1.0;
    private static final double REVERSE_POWER = -1.0;
    private static final double STOP_POWER = 0.0;

    private IntakeMotor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static IntakeMotor getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new IntakeMotor(telemetry);
        }
        return INSTANCE;
    }

    public static IntakeMotor getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException(
                    "IntakeMotor must be initialized with telemetry first!");
        }
        return INSTANCE;
    }

    /**
     * Forward / Stop toggle
     */
    public final Command startStop = new InstantCommand(() -> {

        if (intakeMotor.getPower() > 0.1) {
            // Already running forward -> stop
            intakeMotor.setPower(STOP_POWER);
        } else {
            // Stopped or reversing -> run forward
            intakeMotor.setPower(INTAKE_POWER);
        }

    }).requires(this);

    /**
     * Forward Only
     */
    public final Command forward = new InstantCommand(() -> {
        intakeMotor.setPower(INTAKE_POWER);
    }).requires(this);

    /**
     * Reverse Only
     */
    public final Command reverse = new InstantCommand(() -> {
        intakeMotor.setPower(REVERSE_POWER);
    }).requires(this);

    /**
     * Start the intake motor
     */
    public final void start() {
        intakeMotor.setPower(INTAKE_POWER);
    }
    
    /**
     * Stop the intake motor
     */
    public final void stop() {
        intakeMotor.setPower(STOP_POWER);
    }

    @Override
    public void periodic() {

        telemetry.addData("<===== Intake Motor =====>", "");

        telemetry.addData(
                "Power",
                "%.2f",
                intakeMotor.getPower());

        telemetry.addData(
                "Velocity",
                "%.2f",
                intakeMotor.getVelocity());

        telemetry.addData(
                "State",
                intakeMotor.getPower() > 0.1 ? "FORWARD"
                        : intakeMotor.getPower() < -0.1 ? "REVERSE"
                        : "STOPPED");
    }
}
