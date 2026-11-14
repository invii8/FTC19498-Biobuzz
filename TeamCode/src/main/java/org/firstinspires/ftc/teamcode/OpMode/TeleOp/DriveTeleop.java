package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Current Teleop", group = "00")
public class DriveTeleop extends OpMode {
    private static final double TRANSFER_DELAY_SEC = 0.75;

    private MecanumDrive drive;
    private Shooter shooter;
    private DcMotor intake, transfer;
    private Servo limiter;
    private Servo hood;
    private final ElapsedTime limiterTimer = new ElapsedTime();
    private boolean fieldCentric = false;
    private boolean intakeOn = false;
    private boolean shooting = false;
    private boolean limiterOpen = false;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);

        intake   = hardwareMap.get(DcMotor.class, Constant.INTAKE);
        transfer = hardwareMap.get(DcMotor.class, Constant.TRANSFER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        limiter = hardwareMap.get(Servo.class, Constant.LIMITER);
        limiter.setPosition(Constant.LIMITER_CLOSED);

        hood = hardwareMap.get(Servo.class, Constant.ADJUSTABLE);
        hood.setPosition(Constant.HOOD_MIN_POS);
    }

    @Override
    public void loop() {
        drive.update();

        if (gamepad1.aWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.yWasPressed()) drive.resetHeading();
        if (gamepad1.leftBumperWasPressed()) intakeOn = !intakeOn;
        if (gamepad1.rightBumperWasPressed()) {
            shooting = !shooting;
            limiterOpen = false;
        }

        if (gamepad1.dpadUpWasPressed())   Constant.SHOOTER_SHOOT_RPM += 100;
        if (gamepad1.dpadDownWasPressed()) Constant.SHOOTER_SHOOT_RPM = Math.max(0, Constant.SHOOTER_SHOOT_RPM - 100);

        intake.setPower(intakeOn ? 1.0 : 0.0);

        shooter.setTargetRpm(shooting ? Constant.SHOOTER_SHOOT_RPM : Constant.SHOOTER_IDLE_RPM);
        shooter.update();

        hood.setPosition(shooting ? Constant.HOOD_MAX_POS : Constant.HOOD_MIN_POS);

        if (shooting && !limiterOpen && shooter.atSpeed()) {
            limiterOpen = true;
            limiterTimer.reset();
        }
        limiter.setPosition(limiterOpen ? Constant.LIMITER_OPEN : Constant.LIMITER_CLOSED);

        boolean transferOn = limiterOpen && limiterTimer.seconds() >= TRANSFER_DELAY_SEC && shooter.atSpeed();
        transfer.setPower(transferOn ? 1.0 : 0.0);

        drive.drive(-gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                fieldCentric);

        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Intake", intakeOn);
        telemetry.addData("Transfer", transferOn);
        telemetry.addData("Shooting", shooting);
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.velocityRpm, shooter.targetRpm);
        telemetry.addData("Shooter Ready", shooter.atSpeed());
        telemetry.addData("Limiter", limiterOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Hood", shooting ? "UP" : "DOWN");
        telemetry.addData("Drive Pos", "X=%.1f  Y=%.1f", drive.botX, drive.botY);
        telemetry.addData("Robot Heading", "%.2f", drive.headingDeg);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        shooter.stop();
        intake.setPower(0);
        transfer.setPower(0);
    }
}