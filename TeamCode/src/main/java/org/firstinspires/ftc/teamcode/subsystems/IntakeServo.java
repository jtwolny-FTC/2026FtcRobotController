package org.firstinspires.ftc.teamcode.subsystems;
 
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
 
public class IntakeServo implements Subsystem {
 
private static IntakeServo INSTANCE = null;
 
private final Telemetry telemetry;
 
private IntakeServo(Telemetry telemetry) {
this.telemetry = telemetry;
}
 
public static IntakeServo getInstance(Telemetry telemetry) {
if (INSTANCE == null) {
INSTANCE = new IntakeServo(telemetry);
}
return INSTANCE;
}
 
public static IntakeServo getInstance() {
if (INSTANCE == null) {
throw new NullPointerException(
"IntakeServo subsystem is not initialized yet");
}
return INSTANCE;
}
 
// Left and right intake servos
private final CRServoEx leftIntakeServo =
new CRServoEx("leftIntakeServo");
 
private final CRServoEx rightIntakeServo =
new CRServoEx("rightIntakeServo");
 
private static final double REVERSE_POWER = -1.0;
private static final double STOP_POWER = 0.0;
private static final double INTAKE_POWER = 1.0;
 
private double currentPower = 0.0;
 
/**
* Sets both intake servos.
* Right servo is reversed because it is mounted opposite the left.
*/
private void setIntakePower(double power) {
leftIntakeServo.setPower(power);
rightIntakeServo.setPower(-power);
}
 
/**
* Forward / Stop toggle
*
* Stopped -> Forward
* Forward -> Stop
* Reverse -> Forward
*/
public final Command startStop = new InstantCommand(() -> {
 
if (currentPower > 0.1) {
setIntakePower(STOP_POWER);
currentPower = STOP_POWER;
} else {
setIntakePower(INTAKE_POWER);
currentPower = INTAKE_POWER;
}
 
}).requires(this);
 
/**
* Reverse / Stop toggle
*
* Stopped -> Reverse
* Reverse -> Stop
* Forward -> Reverse
*/
public final Command reverse = new InstantCommand(() -> {
 
if (currentPower < -0.1) {
setIntakePower(STOP_POWER);
currentPower = STOP_POWER;
} else {
setIntakePower(REVERSE_POWER);
currentPower = REVERSE_POWER;
}
 
}).requires(this);
 
/**
* Stop intake.
*/
public void stop() {
setIntakePower(STOP_POWER);
currentPower = STOP_POWER;
}
 
@Override
public void periodic() {
 
telemetry.addData("<===== Intake Subsystem =====>", "");
 
telemetry.addData(
"Left Intake Power",
"%.2f",
currentPower);
 
telemetry.addData(
"Right Intake Power",
"%.2f",
-currentPower);
 
telemetry.addData(
"State",
currentPower > 0.1 ? "FORWARD"
: currentPower < -0.1 ? "REVERSE"
: "STOPPED");
}
}
