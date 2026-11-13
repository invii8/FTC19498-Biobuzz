package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;

@TeleOp(name = "Shooter Tuner", group = "99")
public class ShooterTuner extends OpMode {
    private Shooter shooter;
    private boolean running = false;
    private boolean feeding = false;
    private double avgRpm = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private Servo limiter;
    private DcMotor intake, transfer;

    @Override
    public void init() {
        limiter = hardwareMap.get(Servo.class, Constant.LIMITER);
        limiter.setPosition(Constant.LIMITER_OPEN);

        intake   = hardwareMap.get(DcMotor.class, Constant.INTAKE);
        transfer = hardwareMap.get(DcMotor.class, Constant.TRANSFER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) running = true;
        if (gamepad1.bWasPressed()) running = false;
        if (gamepad1.leftBumperWasPressed()) feeding = !feeding;
        if (gamepad1.dpadUpWasPressed())   Constant.SHOOTER_TARGET_RPM += 100;
        if (gamepad1.dpadDownWasPressed()) Constant.SHOOTER_TARGET_RPM = Math.max(0, Constant.SHOOTER_TARGET_RPM - 100);

        shooter.setTargetRpm(running ? Constant.SHOOTER_TARGET_RPM : 0);
        shooter.update();

        double feedPower = feeding ? 1.0 : 0.0;
        intake.setPower(feedPower);
        transfer.setPower(feedPower);

        avgRpm = avgRpm + 0.05 * (shooter.velocityRpm - avgRpm);

        telemetry.addData("running", running);
        telemetry.addData("feeding", feeding);
        telemetry.addData("targetRPM", shooter.targetRpm);
        telemetry.addData("actualRPM", shooter.velocityRpm);
        telemetry.addData("avgRPM", avgRpm);
        telemetry.addData("errorRPM", shooter.targetRpm - shooter.velocityRpm);
        telemetry.addData("avgErrorRPM", shooter.targetRpm - avgRpm);
        telemetry.addData("atSpeed", shooter.atSpeed());
        telemetry.addData("boosting", shooter.boosting);
        telemetry.addData("power", shooter.power);
        telemetry.addData("ff", shooter.ff);
        telemetry.addData("pid", shooter.pid);
        telemetry.addData("voltage", shooter.voltage);
        telemetry.addData("loopMs", "%.1f", loopTimer.milliseconds());
        telemetry.addData("controls", "A run  B stop  LB feed  DPAD +/-100 RPM");
        telemetry.update();
        loopTimer.reset();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.setPower(0);
        transfer.setPower(0);
    }
}