package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeServo implements Subsystem {

    private static IntakeServo INSTANCE = null;

    private final Telemetry telemetry;

    // Left and right positional intake servos
    private final ServoEx leftIntakeServo =
            new ServoEx("leftIntakeServo");

    private final ServoEx rightIntakeServo =
            new ServoEx("rightIntakeServo");

    /*
     * Intake servo positions.
     *
     * goBILDA servo has approximately 300 degrees of total travel.
     *
     * 0.00 -> starting/retracted position
     * 0.60 -> approximately 180 degrees from starting position
     */
    private static final double RETRACT_POSITION = 0.00;
    private static final double ADVANCE_POSITION = 0.70;

    // Tracks the commanded LEFT servo position
    private double currentPosition = RETRACT_POSITION;

    private IntakeServo(Telemetry telemetry) {

        this.telemetry = telemetry;

        // Start with intake retracted
        retract();
    }

    /**
     * Create/get subsystem instance.
     */
    public static IntakeServo getInstance(Telemetry telemetry) {

        if (INSTANCE == null) {
            INSTANCE = new IntakeServo(telemetry);
        }

        return INSTANCE;
    }

    /**
     * Get existing subsystem instance.
     */
    public static IntakeServo getInstance() {

        if (INSTANCE == null) {
            throw new IllegalStateException(
                    "IntakeServo subsystem is not initialized yet");
        }

        return INSTANCE;
    }

    /**
     * Sets both intake servos.
     *
     * The servos are mechanically mirrored.
     *
     * Left Servo  = requested position
     * Right Servo = 1.0 - requested position
     */
    private void setIntakePosition(double position) {

        // Clamp position to valid servo range
        position = Math.max(
                0.0,
                Math.min(1.0, position));

        currentPosition = position;

        // Left servo
        leftIntakeServo.setPosition(position);

        // Right servo moves in the opposite direction
        rightIntakeServo.setPosition(1.0 - position);
    }

    /**
     * Advance the intake approximately 180 degrees.
     *
     * Can be called directly from Pedro IVY autonomous.
     *
     * Left Servo:  0.00 -> 0.60
     * Right Servo: 1.00 -> 0.40
     */
    public void advance() {

        setIntakePosition(ADVANCE_POSITION);
    }

    /**
     * Retract the intake approximately 180 degrees.
     *
     * Can be called directly from Pedro IVY autonomous.
     *
     * Left Servo:  0.60 -> 0.00
     * Right Servo: 0.40 -> 1.00
     */
    public void retract() {

        setIntakePosition(RETRACT_POSITION);
    }

    /**
     * NextFTC TeleOp command:
     * Advance intake.
     */
    public final Command advanceCommand =
            new InstantCommand(this::advance)
                    .requires(this);

    /**
     * NextFTC TeleOp command:
     * Retract intake.
     */
    public final Command retractCommand =
            new InstantCommand(this::retract)
                    .requires(this);

    /**
     * Periodic telemetry.
     */
    @Override
    public void periodic() {

        double rightPosition =
                1.0 - currentPosition;

        telemetry.addData(
                "<===== Intake Servo =====>",
                "");

        telemetry.addData(
                "Left Position",
                "%.2f",
                currentPosition);

        telemetry.addData(
                "Right Position",
                "%.2f",
                rightPosition);

        telemetry.addData(
                "State",
                Math.abs(
                        currentPosition
                                - ADVANCE_POSITION) < 0.01
                        ? "ADVD"
                        : Math.abs(
                                currentPosition
                                        - RETRACT_POSITION) < 0.01
                        ? "RETD"
                        : "MID");
    }
}


