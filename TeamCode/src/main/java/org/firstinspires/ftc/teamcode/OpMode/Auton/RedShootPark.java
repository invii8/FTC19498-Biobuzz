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
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;

@Config
@Autonomous(name = "🔴 SHOOT PARK", group = "00", preselectTeleOp = "Tele Of All Time")
public class RedShootPark extends OpMode {

    public static double PRELOAD_RPM = 3625;
    public static double RUN_SPEED   = 0.5;

    public static double SHOOT_TIME_SEC     = 2.5;
    public static double SPINUP_TIMEOUT_SEC = 3.0;
    public static double PATH_TIMEOUT_SEC   = 5.0;

    public class Paths {
        public Path MoveForward;
        public Path Park;

        public Paths() {
            MoveForward = line(
                    poseFactory.of(59, 5.1276, 90),
                    poseFactory.of(59, 9.9309, 90)
            ).tangent();

            Park = curve(
                    poseFactory.of(59, 9.9309, 90),
                    poseFactory.of(19.7089, 47.8872, 0),
                    poseFactory.of(9.7724, 94.8031, 101.9582)
            ).tangent();
        }
    }

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose START_POS = poseFactory.of(59, 5.1276, 90);

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
                    follow(paths.MoveForward, RUN_SPEED);
                    setPathState(10);
                }
                break;

            case 10:
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    follow(paths.Park, RUN_SPEED);
                    setPathState(20);
                }
                break;

            case 20:
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
        telemetry.addData("Auton", "RED SHOOT PARK");
        telemetry.addData("Follower", follower == null ? "NOT SET UP (Constants.create returns null)" : "ready");
        telemetry.addData("Preload RPM", "%.0f", PRELOAD_RPM);
        telemetry.addData("Start", "X=%.1f  Y=%.1f  H=%.0f", START_POS.x(), START_POS.y(), Math.toDegrees(START_POS.heading()));
        telemetry.addData("Run Speed", "%.2f", RUN_SPEED);
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
        ramp.setPosition(Constant.RAMP_UP_POS);
        limiter.setPosition(limiterOpen ? Constant.LIMITER_OPEN : Constant.LIMITER_CLOSED);
        intake.setPower(feeding ? 1.0 : 0.0);
        transfer.setPower(feeding ? 1.0 : 0.0);

        Pose p = follower.pose();
        telemetry.addData("Path State",  pathState);
        telemetry.addData("Shoot Stage", shootStage);
        telemetry.addData("Path Busy",   follower.isBusy());
        telemetry.addData("Mode",        follower.mode());
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.velocityRpm, shooter.targetRpm);
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

    private void spinUp(double rpm) {
        targetRpm = rpm;
        hoodUp = true;
    }

    private void startShoot(double rpm) {
        spinUp(rpm);
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