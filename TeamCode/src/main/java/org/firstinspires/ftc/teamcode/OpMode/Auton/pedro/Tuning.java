package org.firstinspires.ftc.teamcode.OpMode.Auton.pedro;

import com.pedropathing.tuning.autotune.*;
import org.firstinspires.ftc.teamcode.OpMode.Auton.pedro.procedures.MecanumTuner;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
}