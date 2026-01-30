package org.firstinspires.ftc.teamcode.Math;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class PIDFVelocityController {
    public double kP, kI, kD;
    public double kS, kV, kA;
    public double maxPower = 1.0;
    public double maxI = 0.5;
    public double slewRate = 4.0;

    private double targetTps = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTargetTps = 0.0;
    private double lastPower = 0.0;
    private double velFilt = 0.0;
    public  double alpha;
    private final VoltageSensor battery;


    public PIDFVelocityController(double kP, double kI, double kD,
                                  double kS, double kV, double kA,
                                  double alpha, VoltageSensor battery) {
        this.kP = kP; this.kI = kI; this.kD = kD;
        this.kS = kS; this.kV = kV; this.kA = kA;
        this.alpha = Math.max(0.0, Math.min(1.0, alpha));
        this.battery = battery;
    }

    public void setTargetTps(double tps) { targetTps = tps; }
    public double getTargetTps() { return targetTps; }

    public double update(double measuredTps, double dtSec) {
        if (dtSec <= 0) return lastPower;
        velFilt += alpha * (measuredTps - velFilt);
        double error = targetTps - velFilt;
        double targetAcc = (targetTps - lastTargetTps) / dtSec;
        double ff = (targetTps > 0 ? 1 : targetTps < 0 ? -1 : 0) * kS + kV * targetTps + kA * targetAcc;
        integral += error * dtSec;
        integral = clamp(integral, -maxI, maxI);
        double deriv = (error - lastError) / dtSec;
        double output = ff + kP * error + kI * integral + kD * deriv;
        if (battery != null) {
            double v = battery.getVoltage();
            if (v > 1e-3) output *= (12.0 / v);
        }
        output = clamp(output, -maxPower, maxPower);
        double maxStep = slewRate * dtSec;
        output = clamp(output, lastPower - maxStep, lastPower + maxStep);
        lastError = error;
        lastTargetTps = targetTps;
        lastPower = output;
        return output;
    }

    public void reset() {
        integral = 0; lastError = 0;
        lastTargetTps = targetTps; lastPower = 0;
        velFilt = 0;
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}

