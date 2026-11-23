package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Field {
    public static boolean topHive(double robotY, boolean currentlyTop) {
        double diff = robotY - Constant.HIVE_SPLIT_Y;
        if (diff > Constant.HIVE_BUFFER_IN) return true;
        if (diff < -Constant.HIVE_BUFFER_IN) return false;
        return currentlyTop;
    }
    public static double goalX(boolean blue) {
        return blue ? Constant.BLUE_GOAL_X : Constant.RED_GOAL_X;
    }
    public static double goalY(boolean blue, boolean top) {
        if (blue) return top ? Constant.BLUE_GOAL_TOP_Y : Constant.BLUE_GOAL_BOTTOM_Y;
        return top ? Constant.RED_GOAL_TOP_Y : Constant.RED_GOAL_BOTTOM_Y;
    }
    public static double resetX(boolean blue) {
        return blue ? Constant.BLUE_RESET_X : Constant.RED_RESET_X;
    }
    public static double resetY(boolean blue) {
        return blue ? Constant.BLUE_RESET_Y : Constant.RED_RESET_Y;
    }
    public static double resetHeadingDeg(boolean blue) {
        return blue ? Constant.BLUE_RESET_HEADING_DEG : Constant.RED_RESET_HEADING_DEG;
    }

    public static double driverForwardDeg(boolean blue) {
        return blue ? Constant.BLUE_DRIVER_FORWARD_DEG : Constant.RED_DRIVER_FORWARD_DEG;
    }

    public static final double FIELD_SIZE_IN       = 141.5;
    public static final double PEDRO_FIELD_SIZE_IN = 144;
    public static double AUTON_POSE_MAX_AGE_SEC    = 90;

    private static boolean autonPoseSaved = false;
    private static double autonX, autonY, autonHeadingDeg;
    private static long autonSavedAtMs = 0;

    public static void saveAutonPose(boolean blue, double x, double y, double headingDeg) {
        double scale = FIELD_SIZE_IN / PEDRO_FIELD_SIZE_IN;
        x *= scale;
        y *= scale;
        if (blue) {
            x = FIELD_SIZE_IN - x;
            y = FIELD_SIZE_IN - y;
            headingDeg += 180;
        }
        autonX = x;
        autonY = y;
        autonHeadingDeg = ((headingDeg + 180) % 360 + 360) % 360 - 180;
        autonSavedAtMs = System.currentTimeMillis();
        autonPoseSaved = true;
    }

    public static boolean hasAutonPose() {
        return autonPoseSaved && System.currentTimeMillis() - autonSavedAtMs < AUTON_POSE_MAX_AGE_SEC * 1000;
    }

    public static Pose2D takeAutonPose() {
        autonPoseSaved = false;
        return new Pose2D(DistanceUnit.INCH, autonX, autonY, AngleUnit.DEGREES, autonHeadingDeg);
    }
}