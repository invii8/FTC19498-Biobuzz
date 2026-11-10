package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    public DcMotorEx motor;
    private final VoltageSensor battery;
    private final ElapsedTime voltTimer = new ElapsedTime();

    private PIDFController shooterPID;
    private double lastKP, lastKI, lastKD;

    public double targetRpm = 0;
    public double velocityRpm = 0;
    public double power = 0;
    public double voltage;

    public Shooter(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, Constant.SHOOTER);
        motor.setDirection(Constant.SHOOTER_DIR);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        battery = hwMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();

        buildPID();
    }

    private void buildPID() {
        shooterPID = new PIDFController(new PIDCoefficients(Constant.kP, Constant.kI, Constant.kD));
        lastKP = Constant.kP;
        lastKI = Constant.kI;
        lastKD = Constant.kD;
    }

    public void setTargetRpm(double rpm) {
        if (rpm <= 0 && targetRpm > 0) shooterPID.reset();
        targetRpm = Math.max(0, rpm);
    }

    public void update() {
        if (lastKP != Constant.kP || lastKI != Constant.kI || lastKD != Constant.kD) {
            buildPID();
        }

        if (voltTimer.seconds() > 0.25) {
            voltage = battery.getVoltage();
            voltTimer.reset();
        }
        velocityRpm = motor.getVelocity() * 60.0 / Constant.SHOOTER_TICKS_PER_REV;

        if (targetRpm <= 0) {
            power = 0;
        } else {
            double voltageComp = Constant.NOMINAL_VOLTAGE / voltage;
            double ff = (Constant.kV * targetRpm + Constant.kS) * voltageComp;

            shooterPID.setTargetPosition(targetRpm);
            double pidContribution = shooterPID.update(velocityRpm);

            power = ff + pidContribution;
            if (Constant.SHOOTER_BOOST && velocityRpm < targetRpm * (1 - Constant.SHOOTER_BOOST_BELOW)) power = 1;
            power = Math.max(0, Math.min(1, power));
        }

        motor.setPower(power);
    }

    public boolean atSpeed() {
        return targetRpm > 0 && Math.abs(targetRpm - velocityRpm) < Constant.SHOOTER_TOLERANCE_RPM;
    }

    public void stop() {
        setTargetRpm(0);
        power = 0;
        motor.setPower(0);
    }
}