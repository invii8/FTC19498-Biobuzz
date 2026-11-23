package org.firstinspires.ftc.teamcode.OpMode.Auton;

import static com.pedropathing.api.Paths.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpMode.Auton.pedro.Constants;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;

@Config
@Autonomous(name = "⚪ Park Only", group = "00")
public class ParkOnly extends OpMode {

    public static double FORWARD_IN       = 15;
    public static double RUN_SPEED        = 0.5;
    public static double DRIVE_AT_SEC     = 27;
    public static double PATH_TIMEOUT_SEC = 3.0;

    public class Paths {
        public Path DriveForward;

        public Paths() {
            DriveForward = line(
                    poseFactory.of(0, 0, 0),
                    poseFactory.of(FORWARD_IN, 0, 0)
            ).tangent();
        }
    }

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose START_POS = poseFactory.of(0, 0, 0);

    private Follower follower;
    private Paths    paths;
    private final ElapsedTime pathTimer   = new ElapsedTime();
    private final ElapsedTime opmodeTimer = new ElapsedTime();
    private final ElapsedTime loopTimer   = new ElapsedTime();
    private int pathState = -1;

    private double loopAvgMs  = 0;
    private double loopPeakMs = 0;
    private int    loops      = 0;

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (opmodeTimer.seconds() >= DRIVE_AT_SEC) {
                    follow(paths.DriveForward, RUN_SPEED);
                    setPathState(10);
                }
                break;

            case 10:
                if (pathDone(PATH_TIMEOUT_SEC)) {
                    follower.hold(follower.pose());
                    setPathState(99);
                }
                break;

            case 99:
                break;
        }
    }

    @Override
    public void init() {
        hardwareMap.get(Servo.class, Constant.LIMITER).setPosition(Constant.LIMITER_CLOSED);
        hardwareMap.get(Servo.class, Constant.ADJUSTABLE).setPosition(Constant.HOOD_MIN_POS);
        hardwareMap.get(Servo.class, Constant.RAMP).setPosition(Constant.RAMP_UP_POS);

        follower = Constants.create(hardwareMap);
        paths    = new Paths();
        if (follower != null) {
            follower.setPose(START_POS);
            follower.update();
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) Constant.ALLIANCE_BLUE = true;
        if (gamepad1.b) Constant.ALLIANCE_BLUE = false;

        telemetry.addData("Auton", "FORWARD (ANY SIDE)");
        telemetry.addData("Alliance for teleop", Constant.ALLIANCE_BLUE ? "BLUE" : "RED");
        telemetry.addLine("Press X for blue, B for red");
        telemetry.addData("Follower", follower == null ? "NOT SET UP (Constants.create returns null)" : "ready");
        telemetry.addData("Plan", "wait until %.0f s, then forward %.1f in", DRIVE_AT_SEC, FORWARD_IN);
        telemetry.addData("Run Speed", "%.2f", RUN_SPEED);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        loopTimer.reset();
        if (follower != null) follower.setPose(START_POS);
        setPathState(0);
    }

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        loops++;
        if (loops > 5) {
            loopAvgMs  = loopAvgMs == 0 ? loopMs : loopAvgMs + 0.05 * (loopMs - loopAvgMs);
            loopPeakMs = Math.max(loopPeakMs, loopMs);
        }

        if (follower == null) {
            telemetry.addLine("Follower is null. Constants.create() is not finished.");
            telemetry.update();
            return;
        }

        follower.update();
        autonomousPathUpdate();

        Pose p = follower.pose();
        telemetry.addData("Alliance",   Constant.ALLIANCE_BLUE ? "BLUE" : "RED");
        telemetry.addData("Driving in", "%.1f s", Math.max(0, DRIVE_AT_SEC - opmodeTimer.seconds()));
        telemetry.addData("Moved",      "%.1f in", p.x() - START_POS.x());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Busy",  follower.isBusy());
        telemetry.addData("Mode",       follower.mode());
        telemetry.addData("Drive Pos",  "X=%.1f  Y=%.1f", p.x(), p.y());
        telemetry.addData("Heading",    "%.1f", Math.toDegrees(p.heading()));
        telemetry.addData("Loop",       "%.1f ms avg, %.0f ms peak", loopAvgMs, loopPeakMs);
        telemetry.addData("Time",       "%.1f", opmodeTimer.seconds());
        telemetry.update();
    }

    private void follow(Path path, double speed) {
        follower.follow(path.with(Constants.foresightConfig.maxPathSpeed.at(speed)));
    }

    private boolean pathDone(double timeoutSec) {
        return pathTimer.seconds() > 0.1 && (!follower.isBusy() || pathTimer.seconds() > timeoutSec);
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }
}