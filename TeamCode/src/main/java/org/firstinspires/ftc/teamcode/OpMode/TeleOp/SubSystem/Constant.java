package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.TreeMap;

@Config
public class Constant {
    public static final String FRONT_LEFT  = "front_left";
    public static final String FRONT_RIGHT = "front_right";
    public static final String BACK_LEFT   = "back_left";
    public static final String BACK_RIGHT  = "back_right";
    public static boolean ALLIANCE_BLUE = false;
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

    public static double ODO_X_OFFSET_IN = -1.6485;
    public static double ODO_Y_OFFSET_IN = -2.5669;
    public static final boolean ODO_X_REVERSED = false;
    public static final boolean ODO_Y_REVERSED = true;

    public static final String LIMELIGHT = "limelight";

    public static final String ADJUSTABLE = "hood";
    public static final String LIMITER    = "limiter";
    public static final String RAMP       = "ramp";

    public static double LIMITER_OPEN   = 0.85;
    public static double LIMITER_CLOSED = 0.1;

    public static double RAMP_UP_POS          = 0.7;
    public static double RAMP_DOWN_POS        = 0.4;

    public static double HOOD_MIN_POS = 0.435;
    public static double HOOD_MAX_POS = 0.5;
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

    public static final DcMotorSimple.Direction SHOOTER_DIR = DcMotorSimple.Direction.REVERSE;

    public static double kP = 0.001;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.000176;
    public static double NOMINAL_VOLTAGE = 12.8;

    public static boolean SHOOTER_BOOST       = true;
    public static double  SHOOTER_BOOST_BELOW_RPM = 100;

    public static double SHOOTER_IDLE_RPM  = 3700;
    public static double SHOOTER_SHOOT_RPM = 3850;
    public static double SHOOTER_TARGET_RPM    = 3000;
    public static double SHOOTER_TOLERANCE_RPM = 100;
    public static double SHOOTER_TICKS_PER_REV = 28;

    public static double AUTON_LIMITER_DELAY_SEC = 0.55;
    public static double AUTON_SHOOT_TIME_SEC    = 3.0;
    public static double AUTON_SHOOT_TIMEOUT_SEC = 7.0;
    public static double SHOOTER_VEL_ALPHA     = 1.0;

    public static boolean USE_SHOOT_TABLE      = true;
    public static double  SHOOT_TABLE_TRIM_RPM = 0;

    public static final TreeMap<Double, Double> SHOOT_TABLE = new TreeMap<>();
    static {
        SHOOT_TABLE.put(10.0, 3500.0);
        SHOOT_TABLE.put(20.0, 3500.0);
        SHOOT_TABLE.put(30.0, 3500.0);
        SHOOT_TABLE.put(40.0, 3500.0);
        SHOOT_TABLE.put(50.0, 3525.0);
        SHOOT_TABLE.put(60.0, 3625.0);
        SHOOT_TABLE.put(70.0, 3900.0);
    }

    public static double RED_GOAL_X        = 57.53;
    public static double RED_GOAL_TOP_Y    = 79.87;
    public static double RED_GOAL_BOTTOM_Y = 60.88;

    public static double BLUE_GOAL_X        = 84.16;
    public static double BLUE_GOAL_TOP_Y    = 79.87;
    public static double BLUE_GOAL_BOTTOM_Y = 60.88;

    public static double RED_RESET_X           = 7.929;
    public static double RED_RESET_Y           = 135.256;
    public static double RED_RESET_HEADING_DEG = 0;

    public static double BLUE_RESET_X           = 133.571;
    public static double BLUE_RESET_Y           = 6.244;
    public static double BLUE_RESET_HEADING_DEG = 180;

    public static double HIVE_SPLIT_Y       = 70.75;
    public static double HIVE_BUFFER_IN = 2;

    public static double TUNER_HOOD_POS = 0.5;

    public static double RED_DRIVER_FORWARD_DEG  = 0;
    public static double BLUE_DRIVER_FORWARD_DEG = 180;
}