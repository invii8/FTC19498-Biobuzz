//package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class Turret {
//    public Servo left, right;
//
//    public double requestedDeg, targetDeg, servoPos;
//    public double trimDeg = 0;
//    public boolean atLimit = false;
//
//    public Turret(HardwareMap hwMap) {
//        left  = hwMap.get(Servo.class, Constant.TURRET_LEFT);
//        right = hwMap.get(Servo.class, Constant.TURRET_RIGHT);
//        if (Constant.TURRET_RIGHT_REVERSED) right.setDirection(Servo.Direction.REVERSE);
//    }
//
//    public void setAngle(double robotRelativeDeg) {
//        requestedDeg = wrap180(robotRelativeDeg + trimDeg);
//        targetDeg = fitToRange(requestedDeg);
//        atLimit = Math.abs(wrap180(targetDeg - requestedDeg)) > 0.5;
//
//        servoPos = clip(Constant.TURRET_CENTER_POS + targetDeg * Constant.TURRET_POS_PER_DEG);
//        left.setPosition(clip(servoPos - Constant.TURRET_ANTIBACKLASH));
//        right.setPosition(clip(servoPos + Constant.TURRET_ANTIBACKLASH));
//    }
//
//    public void aimAt(double robotX, double robotY, double headingRad, double pointX, double pointY) {
//        setAngle(angleToPoint(robotX, robotY, headingRad, pointX, pointY));
//    }
//
//    public double angleToPoint(double robotX, double robotY, double headingRad, double pointX, double pointY) {
//        double c = Math.cos(headingRad);
//        double s = Math.sin(headingRad);
//        double pivotX = robotX + Constant.TURRET_OFFSET_X_IN * c - Constant.TURRET_OFFSET_Y_IN * s;
//        double pivotY = robotY + Constant.TURRET_OFFSET_X_IN * s + Constant.TURRET_OFFSET_Y_IN * c;
//        double fieldDeg = Math.toDegrees(Math.atan2(pointY - pivotY, pointX - pivotX));
//        return wrap180(fieldDeg - Math.toDegrees(headingRad));
//    }
//
//    public double distanceToPoint(double robotX, double robotY, double headingRad, double pointX, double pointY) {
//        double c = Math.cos(headingRad);
//        double s = Math.sin(headingRad);
//        double pivotX = robotX + Constant.TURRET_OFFSET_X_IN * c - Constant.TURRET_OFFSET_Y_IN * s;
//        double pivotY = robotY + Constant.TURRET_OFFSET_X_IN * s + Constant.TURRET_OFFSET_Y_IN * c;
//        return Math.hypot(pointX - pivotX, pointY - pivotY);
//    }
//
//    private static double fitToRange(double deg) {
//        double min = Constant.TURRET_MIN_DEG;
//        double max = Constant.TURRET_MAX_DEG;
//        double a = ((deg - min) % 360 + 360) % 360 + min;
//        if (a <= max) return a;
//        double pastMax = a - max;
//        double beforeMin = (min + 360) - a;
//        return pastMax < beforeMin ? max : min;
//    }
//
//    private static double wrap180(double deg) {
//        deg = ((deg + 180) % 360 + 360) % 360 - 180;
//        return deg;
//    }
//
//    private static double clip(double v) {
//        return Math.max(0, Math.min(1, v));
//    }
//}