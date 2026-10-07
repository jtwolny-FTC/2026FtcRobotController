package org.firstinspires.ftc.teamcode.pedro;

public class Tuning {
    // Tuners go here
    public static Procedure mecanumTuner() {
        return new MecanumTuner();

    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), null, null);
    }
}
