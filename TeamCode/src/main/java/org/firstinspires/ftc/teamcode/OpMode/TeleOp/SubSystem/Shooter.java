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
    public double usedKP, usedKI, usedKD, usedKF;

    public double targetRpm = 0;
    public double rawRpm = 0;
    public double velocityRpm = 0;
    public double power = 0;
    public double ff = 0;
    public double pid = 0;
    public double voltage;
    public boolean boosting = false;

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
        usedKP = Constant.kP;
        usedKI = Constant.kI;
        usedKD = Constant.kD;
    }

    public void setTargetRpm(double rpm) {
        if (rpm <= 0 && targetRpm > 0) shooterPID.reset();
        targetRpm = Math.max(0, rpm);
    }

    public void update() {
        if (usedKP != Constant.kP || usedKI != Constant.kI || usedKD != Constant.kD) {
            buildPID();
        }
        usedKF = Constant.kF;

        if (voltTimer.seconds() > 0.25) {
            voltage = battery.getVoltage();
            voltTimer.reset();
        }

        rawRpm = motor.getVelocity() * 60.0 / Constant.SHOOTER_TICKS_PER_REV;
        velocityRpm += Constant.SHOOTER_VEL_ALPHA * (rawRpm - velocityRpm);

        if (targetRpm <= 0) {
            ff = 0;
            pid = 0;
            power = 0;
            boosting = false;
            motor.setPower(0);
            return;
        }

        ff = usedKF * targetRpm * (Constant.NOMINAL_VOLTAGE / voltage);

        shooterPID.setTargetPosition(targetRpm);
        pid = shooterPID.update(velocityRpm);

        power = Math.max(0, Math.min(1, ff + pid));

        boosting = Constant.SHOOTER_BOOST && velocityRpm < targetRpm * (1 - Constant.SHOOTER_BOOST_BELOW);
        if (boosting) power = 1;

        motor.setPower(power);
    }

    public boolean atSpeed() {
        return targetRpm > 0 && velocityRpm >= targetRpm - Constant.SHOOTER_TOLERANCE_RPM;
    }

    public void stop() {
        setTargetRpm(0);
        ff = 0;
        pid = 0;
        power = 0;
        boosting = false;
        motor.setPower(0);
    }
}