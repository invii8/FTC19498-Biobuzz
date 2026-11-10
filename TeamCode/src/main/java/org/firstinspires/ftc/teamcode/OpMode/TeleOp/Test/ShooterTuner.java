package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;

@TeleOp(name = "Shooter Tuner", group = "99")
public class ShooterTuner extends OpMode {
    private Shooter shooter;
    private boolean running = false;
    private boolean stepMode = false;
    private boolean high = false;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) running = true;
        if (gamepad1.bWasPressed()) running = false;
        if (gamepad1.yWasPressed()) { stepMode = !stepMode; stepTimer.reset(); }
        if (gamepad1.dpadUpWasPressed())   Constant.SHOOTER_TUNE_RPM += 100;
        if (gamepad1.dpadDownWasPressed()) Constant.SHOOTER_TUNE_RPM = Math.max(0, Constant.SHOOTER_TUNE_RPM - 100);

        if (stepMode && stepTimer.seconds() > Constant.SHOOTER_STEP_TIME) {
            high = !high;
            stepTimer.reset();
        }

        double target = 0;
        if (running) {
            if (stepMode) target = high ? Constant.SHOOTER_STEP_HIGH_RPM : Constant.SHOOTER_STEP_LOW_RPM;
            else target = Constant.SHOOTER_TUNE_RPM;
        }

        shooter.setTargetRpm(target);
        shooter.update();

        telemetry.addData("targetRPM", shooter.targetRpm);
        telemetry.addData("actualRPM", shooter.velocityRpm);
        telemetry.addData("errorRPM", shooter.targetRpm - shooter.velocityRpm);
        telemetry.addData("power", shooter.power);
        telemetry.addData("voltage", shooter.voltage);
        telemetry.addData("atSpeed", shooter.atSpeed());
        telemetry.addData("loopMs", loopTimer.milliseconds());
        telemetry.addData("mode", !running ? "STOPPED" : stepMode ? "STEP" : "HOLD");
        telemetry.addData("controls", "A run  B stop  Y step mode  DPAD +/-100 RPM");
        telemetry.update();
        loopTimer.reset();
    }

    @Override
    public void stop() {
        shooter.stop();
    }
}