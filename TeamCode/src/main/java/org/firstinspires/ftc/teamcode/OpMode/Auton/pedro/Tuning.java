package org.firstinspires.ftc.teamcode.OpMode.Auton.pedro;

import com.pedropathing.tuning.autotune.*;
import org.firstinspires.ftc.teamcode.OpMode.Auton.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.OpMode.Auton.pedro.procedures.PinpointTuner;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }
}