//package org.firstinspires.ftc.teamcode.OpMode.Auton;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
//
//@Config
//@Autonomous(name = "Shoot Only", group = "00", preselectTeleOp = "Current Teleop")
//public class Shoot extends OpMode {
//    public static double LIMITER_DELAY_SEC = 0.75;
//    public static double SHOOT_TIME_SEC    = 3.0;
//    public static double TIMEOUT_SEC       = 7.0;
//
//    private Shooter shooter;
//    private DcMotor intake, transfer;
//    private Servo limiter, hood;
//
//    private boolean limiterOpen = false;
//    private boolean done = false;
//    private final ElapsedTime autoTimer = new ElapsedTime();
//    private final ElapsedTime limiterTimer = new ElapsedTime();
//
//    @Override
//    public void init() {
//        shooter = new Shooter(hardwareMap);
//
//        intake   = hardwareMap.get(DcMotor.class, Constant.INTAKE);
//        transfer = hardwareMap.get(DcMotor.class, Constant.TRANSFER);
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);
//        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
//        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        limiter = hardwareMap.get(Servo.class, Constant.LIMITER);
//        limiter.setPosition(Constant.LIMITER_CLOSED);
//
//        hood = hardwareMap.get(Servo.class, Constant.ADJUSTABLE);
//        hood.setPosition(Constant.HOOD_MIN_POS);
//    }
//
//    @Override
//    public void start() {
//        autoTimer.reset();
//        limiterOpen = false;
//        done = false;
//    }
//
//    @Override
//    public void loop() {
//        boolean feeding = false;
//
//        if (!done) {
//            shooter.setTargetRpm(Constant.SHOOTER_SHOOT_RPM);
//            hood.setPosition(Constant.HOOD_MAX_POS);
//
//            if (!limiterOpen && shooter.atSpeed()) {
//                limiterOpen = true;
//                limiterTimer.reset();
//            }
//
//            feeding = limiterOpen && limiterTimer.seconds() >= LIMITER_DELAY_SEC;
//
//            boolean finished = limiterOpen && limiterTimer.seconds() >= LIMITER_DELAY_SEC + SHOOT_TIME_SEC;
//            if (finished || autoTimer.seconds() >= TIMEOUT_SEC) {
//                done = true;
//                limiterOpen = false;
//                feeding = false;
//                shooter.setTargetRpm(0);
//                hood.setPosition(Constant.HOOD_MIN_POS);
//            }
//        }
//
//        intake.setPower(feeding ? 1.0 : 0.0);
//        transfer.setPower(feeding ? 1.0 : 0.0);
//        limiter.setPosition(limiterOpen ? Constant.LIMITER_OPEN : Constant.LIMITER_CLOSED);
//        shooter.update();
//
//        telemetry.addData("Status", done ? "DONE" : limiterOpen ? "SHOOTING" : "SPINNING UP");
//        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.velocityRpm, shooter.targetRpm);
//        telemetry.addData("Feeding", feeding);
//        telemetry.addData("Time", "%.1f", autoTimer.seconds());
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        shooter.stop();
//        intake.setPower(0);
//        transfer.setPower(0);
//        limiter.setPosition(Constant.LIMITER_CLOSED);
//    }
//}