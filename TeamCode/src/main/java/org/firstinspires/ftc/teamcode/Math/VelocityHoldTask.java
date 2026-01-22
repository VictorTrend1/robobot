package org.firstinspires.ftc.teamcode.Math;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityHoldTask {
    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;
    private final PIDFVelocityController ctrl;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime;

    public static VoltageSensor pickVoltage(HardwareMap hw, String preferred) {
        try {
            if (preferred != null) return hw.get(VoltageSensor.class, preferred);
        } catch (Exception ignored) {}
        return hw.voltageSensor.iterator().hasNext() ? hw.voltageSensor.iterator().next() : null;
    }

    public static void enableBulk(HardwareMap hw) {
        for (LynxModule m : hw.getAll(LynxModule.class)) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public VelocityHoldTask(HardwareMap hw,
                            double kP, double kI, double kD,
                            double kS, double kV, double kA,
                            double alpha, double slewRate, double maxI, double maxPower,
                            double targetTps, String voltageName) {

        enableBulk(hw);

        shooter1 = hw.get(DcMotorEx.class, "shooter1");
        shooter2 = hw.get(DcMotorEx.class, "shooter2");

        initMotor(shooter1);
        initMotor(shooter2);

        // Daca un motor merge invers, lasa linia asta activa.
        // Daca ambele merg corect, comenteaz-o.
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        VoltageSensor batt = pickVoltage(hw, voltageName);

        ctrl = new PIDFVelocityController(kP, kI, kD, kS, kV, kA, alpha, batt);
        ctrl.slewRate = slewRate;
        ctrl.maxI = maxI;
        ctrl.maxPower = maxPower;
        ctrl.setTargetTps(targetTps);
        ctrl.reset();

        timer.reset();
        lastTime = timer.seconds();
    }

    private static void initMotor(DcMotorEx m) {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetTps(double tps) {
        ctrl.setTargetTps(tps);
    }

    public double getTargetTps() {
        return ctrl.getTargetTps();
    }

    public double getCurrentTps() {
        return (shooter1.getVelocity() + shooter2.getVelocity()) / 2.0;
    }

    public double getShooter1Tps() {
        return shooter1.getVelocity();
    }

    public double getShooter2Tps() {
        return shooter2.getVelocity();
    }

    public void step() {
        double now = timer.seconds();
        double dt = now - lastTime;
        if (dt <= 0) dt = 1e-3;

        double measured = getCurrentTps();
        double power = ctrl.update(measured, dt);

        shooter1.setPower(power);
        shooter2.setPower(power);

        lastTime = now;
    }

    public void stop() {
        shooter1.setPower(0);
        shooter2.setPower(0);
    }
}
