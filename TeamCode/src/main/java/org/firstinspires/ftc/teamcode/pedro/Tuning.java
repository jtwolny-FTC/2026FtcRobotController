package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.tuning.autotune.Procedure;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.TwoWheelTuner;

public class Tuning {
    // Tuners go here
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, null), null, null);
    }
    public static Procedure twoWheelTuner() {
        return new TwoWheelTuner();
    }
}
