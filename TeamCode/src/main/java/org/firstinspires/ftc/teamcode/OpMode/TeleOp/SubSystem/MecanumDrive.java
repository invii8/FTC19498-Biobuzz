package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {
    public DcMotor leftFront, rightFront, leftBack, rightBack;
    public GoBildaPinpointDriver pinpoint;

    public double botX, botY, headingDeg, headingRad;

    public MecanumDrive(HardwareMap hwMap) {
        leftFront  = hwMap.get(DcMotor.class, Constant.FRONT_LEFT);
        rightFront = hwMap.get(DcMotor.class, Constant.FRONT_RIGHT);
        leftBack   = hwMap.get(DcMotor.class, Constant.BACK_LEFT);
        rightBack  = hwMap.get(DcMotor.class, Constant.BACK_RIGHT);

        leftFront.setDirection(Constant.FRONT_LEFT_DIR);
        rightFront.setDirection(Constant.FRONT_RIGHT_DIR);
        leftBack.setDirection(Constant.BACK_LEFT_DIR);
        rightBack.setDirection(Constant.BACK_RIGHT_DIR);

        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, Constant.ODO);
        pinpoint.setOffsets(Constant.ODO_X_OFFSET_IN, Constant.ODO_Y_OFFSET_IN, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                Constant.ODO_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Constant.ODO_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();
        botX = pos.getX(DistanceUnit.INCH);
        botY = pos.getY(DistanceUnit.INCH);
        headingRad = pos.getHeading(AngleUnit.RADIANS);
        headingDeg = Math.toDegrees(headingRad);
    }

    public void drive(double y, double x, double rx, boolean fieldCentric) {
        if (fieldCentric) {
            double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
            double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);
            x = rotX;
            y = rotY;
        }
        x *= Constant.STRAFE_MULT;
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        leftFront.setPower( (y + x + rx) / denom);
        leftBack.setPower(  (y - x + rx) / denom);
        rightFront.setPower((y - x - rx) / denom);
        rightBack.setPower( (y + x - rx) / denom);
    }

    public void resetHeading() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, botX, botY, AngleUnit.RADIANS, 0));
    }

    public void stop() {
        drive(0, 0, 0, false);
    }
}