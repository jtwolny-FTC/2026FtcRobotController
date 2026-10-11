package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem {

    private static Intake INSTANCE = null;

    private Telemetry telemetry;

    private Intake(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static Intake getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new Intake(telemetry);
        }
        return INSTANCE;
    }

    public static Intake getInstance() {
        if (INSTANCE == null) {
            throw new NullPointerException("Intake subsystem is not initialized yet");
        }
        return INSTANCE;
    }

    // Continuous Rotation Servo
    private CRServo intakeServo;

    private static final double REVERSE_POWER = -1.0;
    private static final double STOP_POWER = 0.0;
    private static final double INTAKE_POWER = 1.0;

    /**
     * Call during robot initialization.
     */
    public void initialize(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
    }

    /**
     * Toggle intake on/off.
     */
    public Command startStop = new InstantCommand(() -> {
        if (Math.abs(currentPower) < 0.1) {
            intakeServo.setPower(INTAKE_POWER);
            currentPower = INTAKE_POWER;
        } else {
            intakeServo.setPower(STOP_POWER);
            currentPower = STOP_POWER;
        }
    }).requires(this);

    /**
     * Stop intake.
     */
    public void stop() {
        intakeServo.setPower(STOP_POWER);
        currentPower = STOP_POWER;
    }

    /**
     * Reverse intake.
     */
    public Command reverse = new InstantCommand(() -> {
        intakeServo.setPower(REVERSE_POWER);
        currentPower = REVERSE_POWER;
    }).requires(this);

    private double currentPower = 0.0;

    @Override
    public void periodic() {
        telemetry.addData("<===== Intake Subsystem =====>", "");
        telemetry.addData("Servo Power", "%.2f", currentPower);
    }
}
