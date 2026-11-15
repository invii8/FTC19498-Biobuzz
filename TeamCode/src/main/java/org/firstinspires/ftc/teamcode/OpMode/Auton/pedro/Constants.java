package org.firstinspires.ftc.teamcode.OpMode.Auton.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(Constant.FRONT_LEFT);
        c.frontRightName.set(Constant.FRONT_RIGHT);
        c.backLeftName.set(Constant.BACK_LEFT);
        c.backRightName.set(Constant.BACK_RIGHT);
        c.frontLeftDirection.set(Constant.FRONT_LEFT_DIR);
        c.frontRightDirection.set(Constant.FRONT_RIGHT_DIR);
        c.backLeftDirection.set(Constant.BACK_LEFT_DIR);
        c.backRightDirection.set(Constant.BACK_RIGHT_DIR);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(Constant.ODO);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(Constant.ODO_X_OFFSET_IN);
        c.yPodOffset.set(Constant.ODO_Y_OFFSET_IN);
        c.xPodDirection.set(Constant.ODO_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(Constant.ODO_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static Follower create(HardwareMap h) {
        return null;
    }
}