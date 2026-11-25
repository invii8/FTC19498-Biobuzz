package org.firstinspires.ftc.teamcode.OpMode.Auton;

import static com.pedropathing.api.Paths.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
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
public abstract class TwoTipAuton extends OpMode {
    public static double PRELOAD_RPM = 3625;
    public static double SHOT_1_RPM  = 3500;
    public static double SHOT_2_RPM  = 3500;

    public static double DRIVE_SPEED  = 1.0;
    public static double INTAKE_SPEED = 0.75;

    public static double F1_PUSH_X = 7;
    public static double F2_PUSH_Y = 134;

    public static double SPINUP_TIMEOUT_SEC = 3.0;
    public static double FEED_TIME_SEC      = 2.5;
    public static double INTAKE_DWELL_SEC   = 0.5;
    public static double SETTLE_SEC         = 0.6;
    public static double PATH_TIMEOUT_SEC   = 5.0;
    public static double INTAKE_TIMEOUT_SEC = 3.0;
    public static double INTAKE_OFF_AT      = 0.3;
    public static double RAMP_DOWN_AT       = 0.3;

    static final double[] START        = {59,     5.1276,  90};
    static final double[] F1_APPROACH  = {18.58,  46.61,  180};
    static final double[] F2_APPROACH  = {46.77,  120.92,  90};
    static final double[] SHOOT        = {55.12,  123.23, 270};
    static final double[] PARK         = {10.312, 97.0525, 30};

    static final double[] C_TO_F1      = {45.3728, 47.547};
    static final double[] C_TO_SHOOT_1 = {23.3047, 91.7288};
    static final double[] C_TO_F2      = {45.9890, 102.0905};
    static final double[] C_TO_SHOOT_2 = {45.6764, 116.74};

    protected abstract boolean isBlue();

    protected abstract String autonName();

    private enum State {
        SHOOT_PRELOAD,
        TO_FLOWER_1, INTAKE_FLOWER_1, DWELL_FLOWER_1, BACK_OFF_1,
        TO_SHOOT_1, SETTLE_1, SHOOT_1,
        TO_FLOWER_2, INTAKE_FLOWER_2, DWELL_FLOWER_2,
        TO_SHOOT_2, SETTLE_2, SHOOT_2,
        PARK, DONE
    }

    private enum ShotStage { IDLE, SPINUP, LIMITER, FEED }

    private final PoseFactory pf = PoseFactory.degrees();

    private Follower follower;
    private Shooter  shooter;
    private DcMotor  intake, transfer;
    private Servo    limiter, hood, ramp;

    private Path toFlower1, intakeFlower1, backOff1, toShoot1;
    private Path toFlower2, intakeFlower2, toShoot2, park;
    private Pose startPose, shootPose;

    private State     state = State.SHOOT_PRELOAD;
    private ShotStage shot  = ShotStage.IDLE;
    private final ElapsedTime stateTimer  = new ElapsedTime();
    private final ElapsedTime shotTimer   = new ElapsedTime();
    private final ElapsedTime matchTimer  = new ElapsedTime();

    private double  targetRpm   = 0;
    private boolean intakeOn    = false;
    private boolean rampDown    = false;
    private boolean feeding    = false;
    private boolean limiterOpen = false;
    private boolean hoodUp      = false;

    private Pose p(double[] red) {
        return isBlue()
                ? pf.of(144 - red[0], 144 - red[1], red[2] + 180)
                : pf.of(red[0], red[1], red[2]);
    }

    private Pose ctrl(double[] red) {
        return isBlue() ? pf.of(144 - red[0], 144 - red[1], 0) : pf.of(red[0], red[1], 0);
    }

    private void buildPaths() {
        Pose start = p(START), f1a = p(F1_APPROACH);
        Pose f1e = p(new double[]{F1_PUSH_X, F1_APPROACH[1], F1_APPROACH[2]});
        Pose f2a = p(F2_APPROACH);
        Pose f2e = p(new double[]{F2_APPROACH[0], F2_PUSH_Y, F2_APPROACH[2]});
        Pose sh = p(SHOOT), pk = p(PARK);

        toFlower1     = curve(start, ctrl(C_TO_F1), f1a).linear(start, f1a);
        intakeFlower1 = line(f1a, f1e).tangent();
        backOff1      = line(f1e, f1a).reverseTangent();
        toShoot1      = curve(f1a, ctrl(C_TO_SHOOT_1), sh).reverseTangent();
        toFlower2     = curve(sh, ctrl(C_TO_F2), f2a).constant(f2a);
        intakeFlower2 = line(f2a, f2e).constant(f2e);
        toShoot2      = curve(f2e, ctrl(C_TO_SHOOT_2), sh).linear(f2e, sh);
        park          = line(sh, pk).reverseTangent();

        startPose = start;
        shootPose = sh;
    }

    private void updateAuto() {
        switch (state) {
            case SHOOT_PRELOAD:
                if (shotDone()) {
                    follow(toFlower1, DRIVE_SPEED);
                    go(State.TO_FLOWER_1);
                }
                break;

            case TO_FLOWER_1:
                if (follower.completion() >= RAMP_DOWN_AT) rampDown = true;
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    setIntake(true);
                    follow(intakeFlower1, INTAKE_SPEED);
                    go(State.INTAKE_FLOWER_1);
                }
                break;

            case INTAKE_FLOWER_1:
                if (pathDone(INTAKE_TIMEOUT_SEC)) {
                    follower.hold(follower.pose());
                    go(State.DWELL_FLOWER_1);
                }
                break;

            case DWELL_FLOWER_1:
                if (stateTimer.seconds() > INTAKE_DWELL_SEC) {
                    follow(backOff1, DRIVE_SPEED);
                    go(State.BACK_OFF_1);
                }
                break;

            case BACK_OFF_1:
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    spinUp(SHOT_1_RPM);
                    follow(toShoot1, DRIVE_SPEED);
                    go(State.TO_SHOOT_1);
                }
                break;

            case TO_SHOOT_1:
                if (intakeOn && follower.completion() >= INTAKE_OFF_AT) setIntake(false);
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    setIntake(false);
                    follower.hold(shootPose);
                    go(State.SETTLE_1);
                }
                break;

            case SETTLE_1:
                if (stateTimer.seconds() > SETTLE_SEC) {
                    startShot(SHOT_1_RPM);
                    go(State.SHOOT_1);
                }
                break;

            case SHOOT_1:
                if (shotDone()) {
                    follow(toFlower2, DRIVE_SPEED);
                    go(State.TO_FLOWER_2);
                }
                break;

            case TO_FLOWER_2:
                if (follower.completion() >= RAMP_DOWN_AT) rampDown = true;
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    setIntake(true);
                    follow(intakeFlower2, INTAKE_SPEED);
                    go(State.INTAKE_FLOWER_2);
                }
                break;

            case INTAKE_FLOWER_2:
                if (pathDone(INTAKE_TIMEOUT_SEC)) {
                    follower.hold(follower.pose());
                    go(State.DWELL_FLOWER_2);
                }
                break;

            case DWELL_FLOWER_2:
                if (stateTimer.seconds() > INTAKE_DWELL_SEC) {
                    spinUp(SHOT_2_RPM);
                    follow(toShoot2, DRIVE_SPEED);
                    go(State.TO_SHOOT_2);
                }
                break;

            case TO_SHOOT_2:
                if (intakeOn && follower.completion() >= INTAKE_OFF_AT) setIntake(false);
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    setIntake(false);
                    follower.hold(shootPose);
                    go(State.SETTLE_2);
                }
                break;

            case SETTLE_2:
                if (stateTimer.seconds() > SETTLE_SEC) {
                    startShot(SHOT_2_RPM);
                    go(State.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (shotDone()) {
                    follow(park, DRIVE_SPEED);
                    go(State.PARK);
                }
                break;

            case PARK:
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    follower.hold(follower.pose());
                    go(State.DONE);
                }
                break;

            case DONE:
                break;
        }
    }

    private void updateShot() {
        switch (shot) {
            case SPINUP:
                if (shooter.atSpeed() || shotTimer.seconds() > SPINUP_TIMEOUT_SEC) {
                    limiterOpen = true;
                    setShot(ShotStage.LIMITER);
                }
                break;
            case LIMITER:
                if (shotTimer.seconds() > Constant.AUTON_LIMITER_DELAY_SEC) {
                    feeding = true;
                    setShot(ShotStage.FEED);
                }
                break;
            case FEED:
                if (shotTimer.seconds() > FEED_TIME_SEC) {
                    feeding     = false;
                    limiterOpen = false;
                    hoodUp      = false;
                    targetRpm   = 0;
                    setShot(ShotStage.IDLE);
                }
                break;
            case IDLE:
                break;
        }
    }

    @Override
    public void init() {
        Constant.ALLIANCE_BLUE = isBlue();

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
        hood    = hardwareMap.get(Servo.class, Constant.ADJUSTABLE);
        ramp    = hardwareMap.get(Servo.class, Constant.RAMP);
        limiter.setPosition(Constant.LIMITER_CLOSED);
        hood.setPosition(Constant.HOOD_MIN_POS);
        ramp.setPosition(Constant.RAMP_UP_POS);

        buildPaths();

        follower = Constants.create(hardwareMap);
        if (follower != null) {
            follower.setPose(startPose);
            follower.update();
        }
    }

    @Override
    public void init_loop() {
        telemetry.addData("Auton", autonName());
        telemetry.addData("Follower", follower == null ? "NOT SET UP (Constants.create returned null)" : "ready");
        telemetry.addData("Start", "X=%.1f  Y=%.1f  H=%.0f",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.addData("RPM", "preload %.0f / shot1 %.0f / shot2 %.0f", PRELOAD_RPM, SHOT_1_RPM, SHOT_2_RPM);
        telemetry.update();
    }

    @Override
    public void start() {
        matchTimer.reset();
        if (follower == null) return;
        follower.setPose(startPose);
        follower.hold(startPose);
        startShot(PRELOAD_RPM);
        go(State.SHOOT_PRELOAD);
    }

    @Override
    public void loop() {
        if (follower == null) {
            telemetry.addLine("Follower is null. Constants.create() is not finished.");
            telemetry.update();
            return;
        }

        follower.update();
        updateAuto();
        updateShot();

        shooter.setTargetRpm(targetRpm);
        shooter.update();
        hood.setPosition(hoodUp ? Constant.HOOD_MAX_POS : Constant.HOOD_MIN_POS);
        ramp.setPosition(rampDown ? Constant.RAMP_DOWN_POS : Constant.RAMP_UP_POS);
        limiter.setPosition(limiterOpen ? Constant.LIMITER_OPEN : Constant.LIMITER_CLOSED);
        intake.setPower(intakeOn || feeding ? 1.0 : 0.0);
        transfer.setPower(feeding ? 1.0 : 0.0);

        Pose pose = follower.pose();
        Field.saveAutonPose(isBlue(), pose.x(), pose.y(), Math.toDegrees(pose.heading()));

        telemetry.addData("State",       state);
        telemetry.addData("Shot",        shot);
        telemetry.addData("Busy / Done", "%b / %.2f", follower.isBusy(), follower.completion());
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.velocityRpm, shooter.targetRpm);
        telemetry.addData("Intake",      intakeOn || feeding);
        telemetry.addData("Ramp",        rampDown ? "DOWN" : "UP");
        telemetry.addData("Pose",        "X=%.1f  Y=%.1f  H=%.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Time",        "%.1f", matchTimer.seconds());
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
        return stateTimer.seconds() > 0.1 && (!follower.isBusy() || stateTimer.seconds() > timeoutSec);
    }

    private void go(State next) {
        state = next;
        stateTimer.reset();
    }

    private void setIntake(boolean on) {
        intakeOn = on;
        rampDown = on;
    }

    private void spinUp(double rpm) {
        targetRpm = rpm;
        hoodUp    = true;
    }

    private void startShot(double rpm) {
        spinUp(rpm);
        setIntake(false);
        setShot(ShotStage.SPINUP);
    }

    private boolean shotDone() {
        return shot == ShotStage.IDLE;
    }

    private void setShot(ShotStage s) {
        shot = s;
        shotTimer.reset();
    }
}
