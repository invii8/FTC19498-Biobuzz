package org.firstinspires.ftc.teamcode.OpMode.Auton;

import static com.pedropathing.api.Paths.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpMode.Auton.pedro.Constants;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Field;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;

@Config
@Autonomous(name = "🔴 2 TIP", group = "00", preselectTeleOp = "Tele Of All Time")
public class Red2Tip extends OpMode {

    public static double PRELOAD_RPM = 3625;
    public static double SHOOT_1_RPM = 3500;
    public static double SHOOT_2_RPM = 3500;

    public static double RUN_SPEED    = 1.0;
    public static double INTAKE_SPEED = 0.1;

    public static double SHOOT_TIME_SEC     = 2.5;
    public static double SPINUP_TIMEOUT_SEC = 3.0;
    public static double INTAKE_DWELL_SEC   = 0.5;
    public static double INTAKE_TIMEOUT_SEC = 3.0;
    public static double PATH_TIMEOUT_SEC   = 5.0;
    public static double TURN_SETTLE_SEC    = 0.6;
    public static double RAMP_UP_COMPLETION = 0.3;

    public class Paths {
        public Path MoveToFirstFlower;
        public Path IntakeFirstFlower;
        public Path MoveBackAgain;
        public Path MoveToShootPos;
        public Path MoveToSecondFlower;
        public Path IntakeSecondFlower;
        public Path ShootPos;
        public Path Park;

        public Paths() {
            MoveToFirstFlower = curve(
                    poseFactory.of(59, 5.1276, 90),
                    poseFactory.of(45.3728, 47.547, 0),
                    poseFactory.of(18.58, 46.61, 180)
            ).linear(poseFactory.of(59, 5.1276, 90), poseFactory.of(18.58, 46.61, 180));

            IntakeFirstFlower = line(
                    poseFactory.of(18.58, 46.61, 180),
                    poseFactory.of(12, 46.61, 180)
            ).tangent();

            MoveBackAgain = line(
                    poseFactory.of(12, 46.61, 180),
                    poseFactory.of(18.58, 46.61, 180)
            ).reverseTangent();

            MoveToShootPos = curve(
                    poseFactory.of(18.58, 46.61, 180),
                    poseFactory.of(23.3047, 91.7288, 0),
                    poseFactory.of(55.12, 123.23, 270)
            ).reverseTangent();

            MoveToSecondFlower = curve(
                    poseFactory.of(55.12, 123.23, 270),
                    poseFactory.of(45.8983, 119.3244, 0),
                    poseFactory.of(46.77, 125.28, 90)
            ).constant(poseFactory.of(46.77, 125.28, 90));

            IntakeSecondFlower = line(
                    poseFactory.of(46.77, 125.28, 90),
                    poseFactory.of(46.77, 129.75, 90)
            ).constant(poseFactory.of(46.77, 129.75, 90));

            ShootPos = curve(
                    poseFactory.of(46.77, 129.75, 90),
                    poseFactory.of(45.6764, 116.74, 0),
                    poseFactory.of(55.12, 123.23, 270)
            ).linear(poseFactory.of(46.77, 129.75, 90), poseFactory.of(55.12, 123.23, 270));

            Park = line(
                    poseFactory.of(55.12, 123.23, 270),
                    poseFactory.of(10.312, 97.0525, 30.0942)
            ).reverseTangent();
        }
    }

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose START_POS = poseFactory.of(59, 5.1276, 90);
    public final Pose SHOOT_POS = poseFactory.of(55.12, 123.23, 270);

    private Follower follower;
    private Paths    paths;
    private final ElapsedTime pathTimer   = new ElapsedTime();
    private final ElapsedTime opmodeTimer = new ElapsedTime();
    private final ElapsedTime shootTimer  = new ElapsedTime();
    private int pathState  = -1;
    private int shootStage = -1;

    private Shooter shooter;
    private DcMotor intake, transfer;
    private Servo   limiter, hood, ramp;

    private double  targetRpm   = 0;
    private boolean hoodUp      = false;
    private boolean rampDown    = false;
    private boolean intakeOn    = false;
    private boolean feeding     = false;
    private boolean limiterOpen = false;

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.hold(START_POS);
                startShoot(PRELOAD_RPM);
                setPathState(1);
                break;

            case 1:
                if (shootStage == -1) {
                    follow(paths.MoveToFirstFlower, RUN_SPEED);
                    setPathState(10);
                }
                break;

            case 10:
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    setIntake(true);
                    follow(paths.IntakeFirstFlower, INTAKE_SPEED);
                    setPathState(11);
                }
                break;

            case 11:
                if (pathDone(INTAKE_TIMEOUT_SEC)) {
                    follower.hold(follower.pose());
                    setPathState(12);
                }
                break;

            case 12:
                if (pathTimer.seconds() > INTAKE_DWELL_SEC) {
                    follow(paths.MoveBackAgain, RUN_SPEED);
                    setPathState(13);
                }
                break;

            case 13:
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    setIntake(false);
                    spinUp(SHOOT_1_RPM);
                    follow(paths.MoveToShootPos, RUN_SPEED);
                    setPathState(20);
                }
                break;

            case 20:
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    follower.hold(SHOOT_POS);
                    setPathState(21);
                }
                break;

            case 21:
                if (pathTimer.seconds() > TURN_SETTLE_SEC) {
                    startShoot(SHOOT_1_RPM);
                    setPathState(22);
                }
                break;

            case 22:
                if (shootStage == -1) {
                    follow(paths.MoveToSecondFlower, RUN_SPEED);
                    setPathState(30);
                }
                break;

            case 30:
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    setIntake(true);
                    follow(paths.IntakeSecondFlower, INTAKE_SPEED);
                    setPathState(31);
                }
                break;

            case 31:
                if (pathDone(INTAKE_TIMEOUT_SEC)) {
                    follower.hold(follower.pose());
                    setPathState(32);
                }
                break;

            case 32:
                if (pathTimer.seconds() > INTAKE_DWELL_SEC) {
                    spinUp(SHOOT_2_RPM);
                    follow(paths.ShootPos, RUN_SPEED);
                    setPathState(40);
                }
                break;

            case 40:
                if (rampDown && follower.completion() >= RAMP_UP_COMPLETION) {
                    setIntake(false);
                }
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    setIntake(false);
                    follower.hold(SHOOT_POS);
                    setPathState(41);
                }
                break;

            case 41:
                if (pathTimer.seconds() > TURN_SETTLE_SEC) {
                    startShoot(SHOOT_2_RPM);
                    setPathState(42);
                }
                break;

            case 42:
                if (shootStage == -1) {
                    follow(paths.Park, RUN_SPEED);
                    setPathState(50);
                }
                break;

            case 50:
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    follower.hold(follower.pose());
                    setPathState(99);
                }
                break;

            case 99:
                break;
        }
    }

    public void shootUpdate() {
        switch (shootStage) {
            case 0:
                if (shooter.atSpeed() || shootTimer.seconds() > SPINUP_TIMEOUT_SEC) {
                    limiterOpen = true;
                    setShootStage(1);
                }
                break;

            case 1:
                if (shootTimer.seconds() > Constant.AUTON_LIMITER_DELAY_SEC) {
                    feeding = true;
                    setShootStage(2);
                }
                break;

            case 2:
                if (shootTimer.seconds() > SHOOT_TIME_SEC) {
                    feeding     = false;
                    limiterOpen = false;
                    hoodUp      = false;
                    targetRpm   = 0;
                    setShootStage(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
        Constant.ALLIANCE_BLUE = false;

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

        ramp = hardwareMap.get(Servo.class, Constant.RAMP);
        ramp.setPosition(Constant.RAMP_UP_POS);

        follower = Constants.create(hardwareMap);
        paths    = new Paths();
        if (follower != null) {
            follower.setPose(START_POS);
            follower.update();
        }
    }

    @Override
    public void init_loop() {
        telemetry.addData("Auton", "RED 2 TIP");
        telemetry.addData("Follower", follower == null ? "NOT SET UP (Constants.create returns null)" : "ready");
        telemetry.addData("RPMs", "preload %.0f / shot 1 %.0f / shot 2 %.0f", PRELOAD_RPM, SHOOT_1_RPM, SHOOT_2_RPM);
        telemetry.addData("Speeds", "run %.2f / intake %.2f", RUN_SPEED, INTAKE_SPEED);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        if (follower != null) follower.setPose(START_POS);
        setPathState(0);
    }

    @Override
    public void loop() {
        if (follower == null) {
            telemetry.addLine("Follower is null. Constants.create() is not finished.");
            telemetry.update();
            return;
        }

        follower.update();
        autonomousPathUpdate();
        shootUpdate();

        shooter.setTargetRpm(targetRpm);
        shooter.update();
        hood.setPosition(hoodUp ? Constant.HOOD_MAX_POS : Constant.HOOD_MIN_POS);
        ramp.setPosition(rampDown ? Constant.RAMP_DOWN_POS : Constant.RAMP_UP_POS);
        limiter.setPosition(limiterOpen ? Constant.LIMITER_OPEN : Constant.LIMITER_CLOSED);
        intake.setPower(intakeOn || feeding ? 1.0 : 0.0);
        transfer.setPower(feeding ? 1.0 : 0.0);

        Pose p = follower.pose();
        Field.saveAutonPose(false, p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("Path State",  pathState);
        telemetry.addData("Shoot Stage", shootStage);
        telemetry.addData("Path Busy",   follower.isBusy());
        telemetry.addData("Mode",        follower.mode());
        telemetry.addData("Completion",  "%.2f", follower.completion());
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.velocityRpm, shooter.targetRpm);
        telemetry.addData("Ramp",        rampDown ? "DOWN" : "UP");
        telemetry.addData("Intake",      intakeOn || feeding);
        telemetry.addData("Feeding",     feeding);
        telemetry.addData("Drive Pos",   "X=%.1f  Y=%.1f", p.x(), p.y());
        telemetry.addData("Heading",     "%.1f", Math.toDegrees(p.heading()));
        telemetry.addData("Time",        "%.1f", opmodeTimer.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.setPower(0);
        transfer.setPower(0);
        limiter.setPosition(Constant.LIMITER_CLOSED);
    }

    private void follow(Path path, double speed) {
        follower.follow(path.with(Constants.foresightConfig.maxPathSpeed.at(speed)));
    }

    private boolean pathDone(double timeoutSec) {
        return pathTimer.seconds() > 0.1 && (!follower.isBusy() || pathTimer.seconds() > timeoutSec);
    }

    private void setIntake(boolean on) {
        rampDown = on;
        intakeOn = on;
    }

    private void spinUp(double rpm) {
        targetRpm = rpm;
        hoodUp = true;
    }

    private void startShoot(double rpm) {
        spinUp(rpm);
        setIntake(false);
        setShootStage(0);
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    private void setShootStage(int stage) {
        shootStage = stage;
        shootTimer.reset();
    }
}