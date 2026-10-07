package org.firstinspires.ftc.teamcode.pedro;

import org.firstinspires.ftc.teamcode.pedro.procedures.TwoWheelTuner;

public class Tuning {
    // Tuners go here
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), null, null);
    }
    public static Procedure twoWheelTuner() {
        return new TwoWheelTuner();
    }
}
