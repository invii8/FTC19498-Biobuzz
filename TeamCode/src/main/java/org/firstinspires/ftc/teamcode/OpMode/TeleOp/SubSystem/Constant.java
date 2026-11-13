package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class Constant {
    public static final String FRONT_LEFT  = "front_left";
    public static final String FRONT_RIGHT = "front_right";
    public static final String BACK_LEFT   = "back_left";
    public static final String BACK_RIGHT  = "back_right";

    public static final DcMotorSimple.Direction FRONT_LEFT_DIR  = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_LEFT_DIR   = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_RIGHT_DIR  = DcMotorSimple.Direction.FORWARD;

    public static double STRAFE_MULT = 1.1;

    public static final String INTAKE    = "intake";
    public static final String TRANSFER  = "transfer";
    public static final String SHOOTER   = "shooter";

    public static final String[] OTHER_MOTORS = { INTAKE, TRANSFER, SHOOTER };

    public static final String ODO = "pinpoint";

    public static double ODO_X_OFFSET_IN = 0;
    public static double ODO_Y_OFFSET_IN = 0;
    public static final boolean ODO_X_REVERSED = false;
    public static final boolean ODO_Y_REVERSED = false;

    public static final String LIMELIGHT = "limelight";

    public static final String ADJUSTABLE = "hood";
    public static final String LIMITER    = "limiter";
    public static final String RAMP       = "ramp";

    public static double LIMITER_OPEN   = 0.95;
    public static double LIMITER_CLOSED = 0.01;

    public static double HOOD_MIN_POS = 0.3;
    public static double HOOD_MAX_POS = 0.8;
    public static final String TURRET_LEFT  = "turret_left";
    public static final String TURRET_RIGHT = "turret_right";

    public static final boolean TURRET_RIGHT_REVERSED = false;

    public static double TURRET_CENTER_POS   = 0.5;
    public static double TURRET_POS_PER_DEG  = 0.9135 / 360.0;
    public static double TURRET_MIN_DEG      = -90;
    public static double TURRET_MAX_DEG      = 90;
    public static double TURRET_ANTIBACKLASH = 0.001;
    public static double TURRET_OFFSET_X_IN  = 0;
    public static double TURRET_OFFSET_Y_IN  = 0;

    public static final DcMotorSimple.Direction SHOOTER_DIR = DcMotorSimple.Direction.FORWARD;

    public static double kP = 0.001;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.000176;
    public static double NOMINAL_VOLTAGE = 12.8;

    public static boolean SHOOTER_BOOST       = true;
    public static double  SHOOTER_BOOST_BELOW = 0.10;

    public static double SHOOTER_IDLE_RPM  = 4000;
    public static double SHOOTER_SHOOT_RPM = 4300;
    public static double SHOOTER_TARGET_RPM    = 3000;
    public static double SHOOTER_TOLERANCE_RPM = 100;
    public static double SHOOTER_TICKS_PER_REV = 28;
    public static double SHOOTER_VEL_ALPHA     = 1.0;
}