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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Field;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "Shooter Tuner", group = "99")
public class ShooterTuner extends OpMode {
    private Shooter shooter;
    private MecanumDrive drive;
    private boolean running = false;
    private boolean feeding = false;
    private boolean topHive = true;
    private double avgRpm = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final List<String> saved = new ArrayList<>();

    private Servo limiter, hood;
    private DcMotor intake, transfer;

    @Override
    public void init() {
        limiter = hardwareMap.get(Servo.class, Constant.LIMITER);
        limiter.setPosition(Constant.LIMITER_OPEN);

        hood = hardwareMap.get(Servo.class, Constant.ADJUSTABLE);
        hood.setPosition(Constant.TUNER_HOOD_POS);

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
        drive = new MecanumDrive(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) Constant.ALLIANCE_BLUE = true;
        if (gamepad1.bWasPressed()) Constant.ALLIANCE_BLUE = false;
        telemetry.addData("Alliance", Constant.ALLIANCE_BLUE ? "BLUE" : "RED");
        telemetry.addLine("X = blue   B = red");
        telemetry.update();
    }

    @Override
    public void start() {
        gamepad1.xWasPressed();
        gamepad1.bWasPressed();
    }

    @Override
    public void loop() {
        boolean blue = Constant.ALLIANCE_BLUE;
        drive.update();

        if (gamepad1.aWasPressed()) running = true;
        if (gamepad1.bWasPressed()) running = false;
        if (gamepad1.leftBumperWasPressed()) feeding = !feeding;
        if (gamepad1.dpadUpWasPressed())   Constant.SHOOTER_TARGET_RPM += 50;
        if (gamepad1.dpadDownWasPressed()) Constant.SHOOTER_TARGET_RPM = Math.max(0, Constant.SHOOTER_TARGET_RPM - 50);
        if (gamepad1.dpadRightWasPressed()) Constant.TUNER_HOOD_POS = Math.min(1, Constant.TUNER_HOOD_POS + 0.005);
        if (gamepad1.dpadLeftWasPressed())  Constant.TUNER_HOOD_POS = Math.max(0, Constant.TUNER_HOOD_POS - 0.005);

        if (gamepad1.xWasPressed()) {
            drive.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,
                    Field.resetX(blue), Field.resetY(blue), AngleUnit.DEGREES, Field.resetHeadingDeg(blue)));
        }

        topHive = Field.topHive(drive.botY, topHive);
        double goalDist = Math.hypot(Field.goalX(blue) - drive.botX, Field.goalY(blue, topHive) - drive.botY);

        if (gamepad1.yWasPressed()) {
            String line = String.format(Locale.US, "SHOOT_TABLE.put(%.1f, %.1f);   hood=%.3f",
                    goalDist, Constant.SHOOTER_TARGET_RPM, Constant.TUNER_HOOD_POS);
            saved.add(line);
            RobotLog.ii("ShooterTuner", line);
        }
        if (gamepad1.rightBumperWasPressed() && !saved.isEmpty()) saved.remove(saved.size() - 1);

        hood.setPosition(Constant.TUNER_HOOD_POS);

        shooter.setTargetRpm(running ? Constant.SHOOTER_TARGET_RPM : 0);
        shooter.update();

        double feedPower = feeding ? 1.0 : 0.0;
        intake.setPower(feedPower);
        transfer.setPower(feedPower);

        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);

        avgRpm = avgRpm + 0.05 * (shooter.velocityRpm - avgRpm);

        telemetry.addData("controls", "A run  B stop  LB feed  DPAD U/D rpm  DPAD L/R hood  X corner reset  Y save point  RB undo");
        telemetry.addData("alliance", blue ? "BLUE" : "RED");
        telemetry.addData("hive", topHive ? "TOP" : "BOTTOM");
        telemetry.addData("goalDist", goalDist);
        telemetry.addData("hoodPos", Constant.TUNER_HOOD_POS);
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
        telemetry.addData("x", drive.botX);
        telemetry.addData("y", drive.botY);
        telemetry.addData("loopMs", "%.1f", loopTimer.milliseconds());
        telemetry.addLine("--- saved points (" + saved.size() + ") ---");
        for (String line : saved) telemetry.addLine(line);
        telemetry.update();
        loopTimer.reset();
    }

    @Override
    public void stop() {
        drive.stop();
        shooter.stop();
        intake.setPower(0);
        transfer.setPower(0);
    }
}