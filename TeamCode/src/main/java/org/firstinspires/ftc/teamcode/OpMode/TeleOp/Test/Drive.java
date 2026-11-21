package org.firstinspires.ftc.teamcode.OpMode.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.MecanumDrive;

@TeleOp(name = "Odo Test", group = "98")
public class Drive extends OpMode {
    private MecanumDrive drive;
    private FtcDashboard dashboard;

    private boolean fieldCentric = false;

    private double lastHeadingDeg = 0;
    private double totalTurnDeg = 0;

    private boolean driftTest = false;
    private double driftStartDeg = 0;
    private final ElapsedTime driftTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new MecanumDrive(hardwareMap);
        telemetry.addLine("Keep the robot STILL during init (gyro calibrating)");
        telemetry.update();
    }

    @Override
    public void start() {
        drive.update();
        lastHeadingDeg = drive.headingDeg;
        totalTurnDeg = 0;
        loopTimer.reset();
    }

    @Override
    public void loop() {
        drive.update();

        double delta = drive.headingDeg - lastHeadingDeg;
        while (delta > 180) delta -= 360;
        while (delta < -180) delta += 360;
        totalTurnDeg += delta;
        lastHeadingDeg = drive.headingDeg;

        if (gamepad1.aWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.yWasPressed()) {
            drive.resetHeading();
            drive.update();
            lastHeadingDeg = drive.headingDeg;
        }
        if (gamepad1.xWasPressed()) {
            drive.pinpoint.resetPosAndIMU();
            totalTurnDeg = 0;
            lastHeadingDeg = 0;
            driftTest = false;
        }
        if (gamepad1.bWasPressed()) {
            driftTest = !driftTest;
            driftStartDeg = totalTurnDeg;
            driftTimer.reset();
        }
        if (gamepad1.dpadDownWasPressed()) totalTurnDeg = 0;

        double scale = gamepad1.right_trigger > 0.5 ? 0.4 : 1.0;
        drive.drive(-gamepad1.left_stick_y * scale,
                gamepad1.left_stick_x * scale,
                gamepad1.right_stick_x * scale,
                fieldCentric);

        telemetry.addData("controls", "A field-centric  Y zero heading  X reset all (still!)  B drift test  DPAD DOWN zero turn count");
        telemetry.addData("fieldCentric", fieldCentric);
        telemetry.addData("pinpointStatus", drive.pinpoint.getDeviceStatus());
        telemetry.addData("x", drive.botX);
        telemetry.addData("y", drive.botY);
        telemetry.addData("heading", drive.headingDeg);
        telemetry.addData("totalTurnDeg", totalTurnDeg);
        telemetry.addData("encoderX", drive.pinpoint.getEncoderX());
        telemetry.addData("encoderY", drive.pinpoint.getEncoderY());
        if (driftTest) {
            double drift = totalTurnDeg - driftStartDeg;
            double minutes = Math.max(driftTimer.seconds() / 60.0, 1e-6);
            telemetry.addData("driftDeg", drift);
            telemetry.addData("driftDegPerMin", drift / minutes);
            telemetry.addData("driftSeconds", driftTimer.seconds());
        }
        telemetry.addData("loopMs", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        double hx = drive.botX + 9 * Math.cos(drive.headingRad);
        double hy = drive.botY + 9 * Math.sin(drive.headingRad);
        packet.fieldOverlay()
                .setStroke("#3F51B5")
                .strokeCircle(drive.botX, drive.botY, 9)
                .strokeLine(drive.botX, drive.botY, hx, hy);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        drive.stop();
    }
}