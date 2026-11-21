//package org.firstinspires.ftc.teamcode.OpMode.TeleOp;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Field;
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.MecanumDrive;
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
//
//import java.util.List;
//
//@Config
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LL only Teleop", group = "00")
//public class LLTele extends OpMode {
//    public static double TRANSFER_DELAY_SEC = 0.75;
//
//    public static int    PIPELINE          = 0;
//    public static double AIM_KP            = 0.02;
//    public static double AIM_KS            = 0.05;
//    public static double AIM_MAX_TURN      = 0.7;
//    public static double AIM_TOLERANCE_DEG = 1.5;
//    public static double AIM_OFFSET_DEG    = 0;
//    public static double DRIVER_TURN_OVERRIDE = 0.2;
//    public static boolean WAIT_FOR_AIM     = true;
//    public static double MAX_STALENESS_MS  = 100;
//
//    private static final int[] RED_TOP_TAGS     = {32, 31};
//    private static final int[] RED_BOTTOM_TAGS  = {35, 36};
//    private static final int[] BLUE_TOP_TAGS    = {44, 43};
//    private static final int[] BLUE_BOTTOM_TAGS = {39, 40};
//
//    private Limelight3A limelight;
//    private boolean tagVisible = false;
//    private String tagIds = "";
//    private double tagTx = 0;
//
//    private MecanumDrive drive;
//    private Shooter shooter;
//    private DcMotor intake, transfer;
//    private Servo limiter;
//    private Servo hood;
//    private final ElapsedTime limiterTimer = new ElapsedTime();
//    private boolean fieldCentric = false;
//    private boolean shooting = false;
//    private boolean limiterOpen = false;
//    private boolean lastShootTrigger = false;
//    private boolean lastBothBumpers = false;
//    private boolean topHive = true;
//
//    @Override
//    public void init() {
//        drive = new MecanumDrive(hardwareMap);
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
//
//        limelight = hardwareMap.get(Limelight3A.class, Constant.LIMELIGHT);
//        limelight.setPollRateHz(100);
//        limelight.pipelineSwitch(PIPELINE);
//        limelight.start();
//    }
//
//    @Override
//    public void init_loop() {
//        if (gamepad1.xWasPressed()) Constant.ALLIANCE_BLUE = true;
//        if (gamepad1.bWasPressed()) Constant.ALLIANCE_BLUE = false;
//
//        telemetry.addData("Alliance", Constant.ALLIANCE_BLUE ? "BLUE" : "RED");
//        telemetry.addLine("Alliance comes from the last auton. Override: X = blue, B = red");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        gamepad1.xWasPressed();
//        gamepad1.bWasPressed();
//    }
//
//    @Override
//    public void loop() {
//        boolean bothBumpers = gamepad1.left_bumper && gamepad1.right_bumper;
//        if (bothBumpers && !lastBothBumpers) Constant.ALLIANCE_BLUE = !Constant.ALLIANCE_BLUE;
//        lastBothBumpers = bothBumpers;
//
//        boolean blue = Constant.ALLIANCE_BLUE;
//        drive.update();
//
//        if (gamepad1.aWasPressed()) fieldCentric = !fieldCentric;
//        if (gamepad1.xWasPressed()) resetToCorner(blue);
//        if (gamepad1.yWasPressed()) setHeading(Field.driverForwardDeg(blue));
//
//        boolean shootTrigger = gamepad1.right_trigger > 0.5;
//        if (shootTrigger && !lastShootTrigger) {
//            shooting = !shooting;
//            limiterOpen = false;
//        }
//        lastShootTrigger = shootTrigger;
//
//        if (gamepad1.dpadUpWasPressed()) {
//            if (Constant.USE_SHOOT_TABLE) Constant.SHOOT_TABLE_TRIM_RPM += 50;
//            else Constant.SHOOTER_SHOOT_RPM += 100;
//        }
//        if (gamepad1.dpadDownWasPressed()) {
//            if (Constant.USE_SHOOT_TABLE) Constant.SHOOT_TABLE_TRIM_RPM -= 50;
//            else Constant.SHOOTER_SHOOT_RPM = Math.max(0, Constant.SHOOTER_SHOOT_RPM - 100);
//        }
//
//        topHive = Field.topHive(drive.botY, topHive);
//        readLimelight(blue, topHive);
//        double goalX = Field.goalX(blue);
//        double goalY = Field.goalY(blue, topHive);
//        double goalDist = Math.hypot(goalX - drive.botX, goalY - drive.botY);
//        double shootRpm = Constant.USE_SHOOT_TABLE
//                ? Shooter.rpmForDistance(goalDist) + Constant.SHOOT_TABLE_TRIM_RPM
//                : Constant.SHOOTER_SHOOT_RPM;
//
//        shooter.setTargetRpm(shooting ? shootRpm : Constant.SHOOTER_IDLE_RPM);
//        shooter.update();
//
//        hood.setPosition(shooting ? Constant.HOOD_MAX_POS : Constant.HOOD_MIN_POS);
//
//        double aimError = tagTx - AIM_OFFSET_DEG;
//        boolean aimed = tagVisible && Math.abs(aimError) <= AIM_TOLERANCE_DEG;
//        boolean driverTurning = Math.abs(gamepad1.right_stick_x) > DRIVER_TURN_OVERRIDE;
//        boolean autoAiming = shooting && tagVisible && !driverTurning;
//
//        double turn = gamepad1.right_stick_x;
//        if (autoAiming) {
//            turn = 0;
//            if (!aimed) {
//                turn = AIM_KP * aimError + Math.signum(aimError) * AIM_KS;
//                turn = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, turn));
//            }
//        }
//
//        boolean aimOk = !WAIT_FOR_AIM || aimed || !tagVisible;
//        if (shooting && !limiterOpen && shooter.atSpeed() && aimOk) {
//            limiterOpen = true;
//            limiterTimer.reset();
//        }
//        limiter.setPosition(limiterOpen ? Constant.LIMITER_OPEN : Constant.LIMITER_CLOSED);
//
//        boolean intakeHeld = gamepad1.left_trigger > 0.5;
//        boolean feeding = limiterOpen && limiterTimer.seconds() >= TRANSFER_DELAY_SEC;
//
//        intake.setPower(intakeHeld || feeding ? 1.0 : 0.0);
//        transfer.setPower(feeding ? 1.0 : 0.0);
//
//        double driveY = -gamepad1.left_stick_y;
//        double driveX = gamepad1.left_stick_x;
//        if (fieldCentric) {
//            double a = drive.headingRad - Math.toRadians(Field.driverForwardDeg(blue));
//            double rotX = driveX * Math.cos(-a) - driveY * Math.sin(-a);
//            double rotY = driveX * Math.sin(-a) + driveY * Math.cos(-a);
//            driveX = rotX;
//            driveY = rotY;
//        }
//        drive.drive(driveY, driveX, turn, false);
//
//        telemetry.addData("Alliance", blue ? "BLUE" : "RED");
//        telemetry.addData("Hive", topHive ? "TOP" : "BOTTOM");
//        telemetry.addData("Goal Dist (in)", "%.1f", goalDist);
//        telemetry.addData("Shoot RPM", "%.0f  (%s)", shootRpm, Constant.USE_SHOOT_TABLE ? "table, trim " + (int) Constant.SHOOT_TABLE_TRIM_RPM : "fixed");
//        telemetry.addData("Tags", tagVisible ? tagIds : "NONE");
//        telemetry.addData("tx", "%.2f", tagTx);
//        telemetry.addData("Aim", !shooting ? "-" : !tagVisible ? "NO TAG (aim by hand)" : driverTurning ? "DRIVER OVERRIDE" : aimed ? "LOCKED" : "TURNING");
//        telemetry.addData("Field Centric", fieldCentric);
//        telemetry.addData("Shooting", shooting);
//        telemetry.addData("Intake", intakeHeld || feeding);
//        telemetry.addData("Transfer", feeding);
//        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.velocityRpm, shooter.targetRpm);
//        telemetry.addData("Shooter Ready", shooter.atSpeed());
//        telemetry.addData("Limiter", limiterOpen ? "OPEN" : "CLOSED");
//        telemetry.addData("Hood", shooting ? "UP" : "DOWN");
//        telemetry.addData("Drive Pos", "X=%.1f  Y=%.1f", drive.botX, drive.botY);
//        telemetry.addData("Robot Heading", "%.2f", drive.headingDeg);
//        telemetry.update();
//    }
//
//    private void readLimelight(boolean blue, boolean top) {
//        tagVisible = false;
//        tagIds = "";
//
//        int[] wanted = blue ? (top ? BLUE_TOP_TAGS : BLUE_BOTTOM_TAGS) : (top ? RED_TOP_TAGS : RED_BOTTOM_TAGS);
//
//        LLResult result = limelight.getLatestResult();
//        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) return;
//
//        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
//        if (tags == null) return;
//
//        double sum = 0;
//        int count = 0;
//        for (LLResultTypes.FiducialResult t : tags) {
//            int id = t.getFiducialId();
//            if (id != wanted[0] && id != wanted[1]) continue;
//            sum += t.getTargetXDegrees();
//            count++;
//            tagIds += (count > 1 ? ", " : "") + id;
//        }
//        if (count == 0) return;
//
//        tagVisible = true;
//        tagTx = sum / count;
//    }
//
//    private void setHeading(double headingDeg) {
//        drive.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, drive.botX, drive.botY, AngleUnit.DEGREES, headingDeg));
//    }
//
//    private void resetToCorner(boolean blue) {
//        drive.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,
//                Field.resetX(blue), Field.resetY(blue), AngleUnit.DEGREES, Field.resetHeadingDeg(blue)));
//    }
//
//    @Override
//    public void stop() {
//        drive.stop();
//        shooter.stop();
//        intake.setPower(0);
//        transfer.setPower(0);
//        limelight.stop();
//    }
//}