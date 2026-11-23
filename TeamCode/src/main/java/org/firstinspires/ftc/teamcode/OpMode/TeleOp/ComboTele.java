package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import static org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant.RAMP_DOWN_POS;
import static org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant.RAMP_UP_POS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Field;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;

import java.util.List;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele Of All Time", group = "00")
public class ComboTele extends OpMode {
    public static double TRANSFER_DELAY_SEC = 0.75;

    public static int    PIPELINE             = 0;
    public static double AIM_KP               = 0.02;
    public static double AIM_KS               = 0.05;
    public static double AIM_MAX_TURN         = 0.5;
    public static double AIM_TOLERANCE_DEG    = 1.5;
    public static double AIM_OFFSET_DEG       = 0;
    public static double ODO_AIM_TOLERANCE_DEG = 3.0;
    public static double ODO_AIM_OFFSET_DEG   = 0;
    public static double DRIVER_TURN_OVERRIDE = 0.2;
    public static boolean WAIT_FOR_AIM        = true;
    public static double MAX_STALENESS_MS     = 100;

    private static final int[] RED_TOP_TAGS     = {32, 31};
    private static final int[] RED_BOTTOM_TAGS  = {35, 36};
    private static final int[] BLUE_TOP_TAGS    = {44, 43};
    private static final int[] BLUE_BOTTOM_TAGS = {39, 40};

    private Limelight3A limelight;
    private boolean limelightEnabled = true;
    private boolean odoEnabled = true;
    private boolean odoCalibrated = false;

    private boolean tagVisible = false;
    private boolean tagIsTop = true;
    private String tagIds = "";
    private double tagTx = 0;

    private MecanumDrive drive;
    private Shooter shooter;
    private DcMotor intake, transfer;
    private Servo limiter;
    private Servo hood;
    private Servo ramp;
    private final ElapsedTime limiterTimer = new ElapsedTime();
    private boolean fieldCentric = false;
    private boolean shooting = false;
    private boolean limiterOpen = false;
    private boolean lastShootTrigger = false;
    private boolean lastBothBumpers = false;
    private boolean rampDown = false;
    private boolean topHive = true;

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

        ramp = hardwareMap.get(Servo.class, Constant.RAMP);
        ramp.setPosition(RAMP_UP_POS);

        limelight = hardwareMap.tryGet(Limelight3A.class, Constant.LIMELIGHT);
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(PIPELINE);
            limelight.start();
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) Constant.ALLIANCE_BLUE = true;
        if (gamepad1.bWasPressed()) Constant.ALLIANCE_BLUE = false;

        telemetry.addData("Alliance", Constant.ALLIANCE_BLUE ? "BLUE" : "RED");
        telemetry.addData("Limelight", limelight == null ? "NOT FOUND" : "found");
        telemetry.addData("Position from auton", Field.hasAutonPose() ? "yes, odometry ready at start" : "none (corner + X)");
        telemetry.update();
    }

    @Override
    public void start() {
        gamepad1.xWasPressed();
        gamepad1.bWasPressed();
        gamepad1.leftBumperWasPressed();

        if (Field.hasAutonPose()) {
            drive.pinpoint.setPosition(Field.takeAutonPose());
            odoCalibrated = true;
        }
    }

    @Override
    public void loop() {
        boolean bothBumpers = gamepad1.left_bumper && gamepad1.right_bumper;
        if (bothBumpers && !lastBothBumpers) Constant.ALLIANCE_BLUE = !Constant.ALLIANCE_BLUE;
        lastBothBumpers = bothBumpers;

        if (gamepad1.leftBumperWasPressed() && !gamepad1.right_bumper) rampDown = !rampDown;

        boolean blue = Constant.ALLIANCE_BLUE;
        drive.update();

        if (gamepad1.aWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.xWasPressed()) {
            resetToCorner(blue);
            odoCalibrated = true;
            odoEnabled = true;
        }
        if (gamepad1.yWasPressed()) setHeading(Field.driverForwardDeg(blue));
        if (gamepad1.dpadLeftWasPressed())  limelightEnabled = !limelightEnabled;
        if (gamepad1.dpadRightWasPressed()) odoEnabled = !odoEnabled;

        boolean shootTrigger = gamepad1.right_trigger > 0.5;
        if (shootTrigger && !lastShootTrigger) {
            shooting = !shooting;
            limiterOpen = false;
        }
        lastShootTrigger = shootTrigger;

        boolean pinpointOk = drive.pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        boolean odoOk = odoEnabled && odoCalibrated && pinpointOk;
        boolean limelightOk = limelightEnabled && limelight != null;

        boolean useTable = Constant.USE_SHOOT_TABLE && odoOk;
        if (gamepad1.dpadUpWasPressed()) {
            if (useTable) Constant.SHOOT_TABLE_TRIM_RPM += 50;
            else Constant.SHOOTER_SHOOT_RPM += 100;
        }
        if (gamepad1.dpadDownWasPressed()) {
            if (useTable) Constant.SHOOT_TABLE_TRIM_RPM -= 50;
            else Constant.SHOOTER_SHOOT_RPM = Math.max(0, Constant.SHOOTER_SHOOT_RPM - 100);
        }

        if (odoOk) topHive = Field.topHive(drive.botY, topHive);

        readLimelight(limelightOk, blue, topHive, !odoOk);
        if (!odoOk && tagVisible) topHive = tagIsTop;

        double goalX = Field.goalX(blue);
        double goalY = Field.goalY(blue, topHive);
        double goalDist = Math.hypot(goalX - drive.botX, goalY - drive.botY);
        double shootRpm = useTable
                ? Shooter.rpmForDistance(goalDist) + Constant.SHOOT_TABLE_TRIM_RPM
                : Constant.SHOOTER_SHOOT_RPM;

        shooter.setTargetRpm(shooting ? shootRpm : Constant.SHOOTER_IDLE_RPM);
        shooter.update();

        hood.setPosition(shooting ? Constant.HOOD_MAX_POS : Constant.HOOD_MIN_POS);

        String aimSource = "NONE";
        double aimError = 0;
        double tolerance = AIM_TOLERANCE_DEG;

        if (tagVisible) {
            aimSource = "LIMELIGHT";
            aimError = tagTx - AIM_OFFSET_DEG;
        } else if (odoOk) {
            aimSource = "ODOMETRY";
            double bearingDeg = Math.toDegrees(Math.atan2(goalY - drive.botY, goalX - drive.botX));
            aimError = -wrap180(bearingDeg - drive.headingDeg) - ODO_AIM_OFFSET_DEG;
            tolerance = ODO_AIM_TOLERANCE_DEG;
        }

        boolean hasAim = !aimSource.equals("NONE");
        boolean aimed = hasAim && Math.abs(aimError) <= tolerance;
        boolean driverTurning = Math.abs(gamepad1.right_stick_x) > DRIVER_TURN_OVERRIDE;
        boolean autoAiming = shooting && hasAim && !driverTurning;

        double turn = gamepad1.right_stick_x;
        if (autoAiming) {
            turn = 0;
            if (!aimed) {
                turn = AIM_KP * aimError + Math.signum(aimError) * AIM_KS;
                turn = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, turn));
            }
        }

        boolean aimOk = !WAIT_FOR_AIM || aimed || !hasAim || driverTurning;
        if (shooting && !limiterOpen && shooter.atSpeed() && aimOk) {
            limiterOpen = true;
            limiterTimer.reset();
        }
        limiter.setPosition(limiterOpen ? Constant.LIMITER_OPEN : Constant.LIMITER_CLOSED);

        boolean intakeHeld = gamepad1.left_trigger > 0.5;
        boolean feeding = limiterOpen && limiterTimer.seconds() >= TRANSFER_DELAY_SEC;

        ramp.setPosition(rampDown ? RAMP_DOWN_POS : RAMP_UP_POS);
        intake.setPower(intakeHeld || feeding || rampDown ? 1.0 : 0.0);
        transfer.setPower(feeding ? 1.0 : 0.0);

        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x;
        if (fieldCentric) {
            double a = drive.headingRad - Math.toRadians(Field.driverForwardDeg(blue));
            double rotX = driveX * Math.cos(-a) - driveY * Math.sin(-a);
            double rotY = driveX * Math.sin(-a) + driveY * Math.cos(-a);
            driveX = rotX;
            driveY = rotY;
        }
        drive.drive(driveY, driveX, turn, false);

        telemetry.addData("Alliance", blue ? "BLUE" : "RED");
        telemetry.addData("Limelight", limelight == null ? "NOT FOUND" : !limelightEnabled ? "OFF (dpad left)" : tagVisible ? "tags " + tagIds : "on, no tags");
        telemetry.addData("Odometry", !pinpointOk ? "PINPOINT FAULT" : !odoEnabled ? "OFF (dpad right)" : !odoCalibrated ? "NOT CALIBRATED (corner + X)" : "ok");
        telemetry.addData("Aim Error (deg)", "%.2f", aimError);
        telemetry.addData("Hive", topHive ? "TOP" : "BOTTOM");
        telemetry.addData("Goal Dist (in)", odoOk ? String.format("%.1f", goalDist) : "n/a");
        telemetry.addData("Shoot RPM", "%.0f  (%s)", shootRpm, useTable ? "table, trim " + (int) Constant.SHOOT_TABLE_TRIM_RPM : "fixed");
        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Intake", intakeHeld || feeding || rampDown);
        telemetry.addData("Transfer", feeding);
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.velocityRpm, shooter.targetRpm);
        telemetry.addData("Drive Pos", "X=%.1f  Y=%.1f", drive.botX, drive.botY);
        telemetry.addData("Robot Heading", "%.2f", drive.headingDeg);
        telemetry.update();
    }

    private void readLimelight(boolean enabled, boolean blue, boolean top, boolean eitherHive) {
        tagVisible = false;
        tagIds = "";
        if (!enabled) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return;

        int[] topTags = blue ? BLUE_TOP_TAGS : RED_TOP_TAGS;
        int[] bottomTags = blue ? BLUE_BOTTOM_TAGS : RED_BOTTOM_TAGS;

        double topSum = 0, bottomSum = 0;
        int topCount = 0, bottomCount = 0;
        String topIds = "", bottomIds = "";

        for (LLResultTypes.FiducialResult t : tags) {
            int id = t.getFiducialId();
            if (id == topTags[0] || id == topTags[1]) {
                topSum += t.getTargetXDegrees();
                topCount++;
                topIds += (topCount > 1 ? ", " : "") + id;
            } else if (id == bottomTags[0] || id == bottomTags[1]) {
                bottomSum += t.getTargetXDegrees();
                bottomCount++;
                bottomIds += (bottomCount > 1 ? ", " : "") + id;
            }
        }

        boolean useTop;
        if (eitherHive) {
            if (topCount == 0 && bottomCount == 0) return;
            if (topCount > 0 && bottomCount > 0) useTop = Math.abs(topSum / topCount) <= Math.abs(bottomSum / bottomCount);
            else useTop = topCount > 0;
        } else {
            useTop = top;
            if ((useTop ? topCount : bottomCount) == 0) return;
        }

        tagVisible = true;
        tagIsTop = useTop;
        tagTx = useTop ? topSum / topCount : bottomSum / bottomCount;
        tagIds = useTop ? topIds : bottomIds;
    }

    private static double wrap180(double deg) {
        return ((deg + 180) % 360 + 360) % 360 - 180;
    }

    private void setHeading(double headingDeg) {
        drive.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, drive.botX, drive.botY, AngleUnit.DEGREES, headingDeg));
    }

    private void resetToCorner(boolean blue) {
        drive.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,
                Field.resetX(blue), Field.resetY(blue), AngleUnit.DEGREES, Field.resetHeadingDeg(blue)));
    }

    @Override
    public void stop() {
        drive.stop();
        shooter.stop();
        intake.setPower(0);
        transfer.setPower(0);
        if (limelight != null) limelight.stop();
    }
}