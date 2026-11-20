package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

// field positions for each alliance, all the numbers live in Constant
public class Field {
    // checks if we're on the top or bottom half of the field
    // the buffer stops it from flipping back and forth when we're right on the line
    public static boolean topHive(double robotY, boolean currentlyTop) {
        double diff = robotY - Constant.HIVE_SPLIT_Y;
        if (diff > Constant.HIVE_BUFFER_IN) return true;
        if (diff < -Constant.HIVE_BUFFER_IN) return false;
        return currentlyTop;
    }
    // goal x for our alliance
    public static double goalX(boolean blue) {
        return blue ? Constant.BLUE_GOAL_X : Constant.RED_GOAL_X;
    }
    // goal y, depends on alliance and which half we're on
    public static double goalY(boolean blue, boolean top) {
        if (blue) return top ? Constant.BLUE_GOAL_TOP_Y : Constant.BLUE_GOAL_BOTTOM_Y;
        return top ? Constant.RED_GOAL_TOP_Y : Constant.RED_GOAL_BOTTOM_Y;
    }
    // where we put the robot when resetting position
    public static double resetX(boolean blue) {
        return blue ? Constant.BLUE_RESET_X : Constant.RED_RESET_X;
    }
    public static double resetY(boolean blue) {
        return blue ? Constant.BLUE_RESET_Y : Constant.RED_RESET_Y;
    }
    public static double resetHeadingDeg(boolean blue) {
        return blue ? Constant.BLUE_RESET_HEADING_DEG : Constant.RED_RESET_HEADING_DEG;
    }

    // which way is "forward" from the driver's side, used for field centric
    public static double driverForwardDeg(boolean blue) {
        return blue ? Constant.BLUE_DRIVER_FORWARD_DEG : Constant.RED_DRIVER_FORWARD_DEG;
    }
}