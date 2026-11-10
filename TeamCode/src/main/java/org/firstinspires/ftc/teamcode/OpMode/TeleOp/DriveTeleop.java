package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drive", group = "00")
public class DriveTeleop extends OpMode {
    private MecanumDrive drive;
    private boolean fieldCentric = false;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        drive.update();

        if (gamepad1.aWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.yWasPressed()) drive.resetHeading();

        double scale = gamepad1.right_trigger > 0.5 ? 0.4 : 1.0;
        drive.drive(-gamepad1.left_stick_y * scale,
                gamepad1.left_stick_x * scale,
                gamepad1.right_stick_x * scale,
                fieldCentric);

        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Slow Mode", scale < 1);
        telemetry.addData("Drive Pos", "X=%.1f  Y=%.1f", drive.botX, drive.botY);
        telemetry.addData("Robot Heading", "%.2f", drive.headingDeg);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}