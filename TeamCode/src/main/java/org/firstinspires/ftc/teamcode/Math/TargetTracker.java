package org.firstinspires.ftc.teamcode.Math;

public final class TargetTracker {

    public static final class Params {
        public double servoCenter = 0.5;

        public double servoLeft;
        public double servoRight;

        public double servoMinLimit = 0.0;
        public double servoMaxLimit = 1.0;

        public double maxYawDeg = 110.0;

        public double txTolDeg = 0.2;
        public int lockFrames = 2;
        public double txAlpha = 0.05;

        public double visionLatencySec = 0.03;

        public double snapMaxServoSpeed = 2.2;
        public double trackMaxServoSpeed = 1.8;

        public double corrMax = 0.40;

        public double kP = -1.3;
        public double kD = -1.6;

        public double enableBiasLearning = 1.0; // 0 disables, 1 enables
        public double headingBiasMaxDeg = 30.0;
        public double centerBiasMax = 0.08;

        public double headingBiasGain = 0.055;
        public double centerBiasGain = 0.0015;

        public double biasUpdateMaxStepDeg = 0.30;
        public double biasUpdateMaxStepServo = 0.0020;

        public double biasUpdateTxWindowDeg = 4.0;
        public double biasUpdateOmegaMaxDegPerSec = 90.0;
        public double biasUpdateLimitMargin = 0.02;

        public double targetX = 0.0;
        public double targetY = 0.0;
    }

    public enum Mode {
        IDLE,
        AIMING,
        LOCKED
    }

    private final Params p;

    private Mode mode = Mode.IDLE;

    private double servoCmd = 0.5;

    private double txF = 0.0;
    private double txPrev = 0.0;

    private double headingRadPrev = 0.0;

    private int lockCount = 0;

    private double headingBiasRad = 0.0;
    private double centerBias = 0.0;
    private double leftBias = 0.0;
    private double rightBias = 0.0;

    private double targetX;
    private double targetY;

    public TargetTracker(Params params) {
        this.p = params;
        this.servoCmd = clamp(params.servoCenter, params.servoMinLimit, params.servoMaxLimit);
        this.targetX = params.targetX;
        this.targetY = params.targetY;
    }

    public Mode getMode() {
        return mode;
    }

    public boolean isLocked() {
        return mode == Mode.LOCKED;
    }

    public double getServoCommand() {
        return servoCmd;
    }

    public void setTarget(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }


    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    public void reset() {
        mode = Mode.IDLE;
        servoCmd = clamp(p.servoCenter, p.servoMinLimit, p.servoMaxLimit);
        txF = 0.0;
        txPrev = 0.0;
        lockCount = 0;
        headingBiasRad = 0.0;
        centerBias = 0.0;
        leftBias = 0.0;
        rightBias = 0.0;
    }

    public void requestAim() {
        mode = Mode.AIMING;
        lockCount = 0;
    }

    public void cancel() {
        mode = Mode.IDLE;
        lockCount = 0;
    }

    public void hold() {
        mode = Mode.LOCKED;
        lockCount = p.lockFrames;
    }

    public double update(
            double x, double y,
            double headingRad,
            boolean hasVision,
            double txDeg,
            double dtSec
    ) {
        if (dtSec <= 1e-6) dtSec = 1e-6;

        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(headingRad)) {
            return 0.5;
        }

        if (Double.isNaN(txDeg)) {
            txDeg = 0.0;
        }
        /// A murit thread-u de 3 ori fara asta , servo isi ia pozitia NaN

        if (p.kP < 0 || p.kD < 0) autoTuneGains();

        double headingDeg = Math.toDegrees(headingRad);
        double headingDegPrev = Math.toDegrees(headingRadPrev);
        double omegaDegPerSec = (headingDeg - headingDegPrev) / dtSec;
        omegaDegPerSec = wrapDeg(omegaDegPerSec);

        headingRadPrev = headingRad;

        if (mode == Mode.IDLE) {
            servoCmd = slewTo(servoCmd, clamp(p.servoCenter, p.servoMinLimit, p.servoMaxLimit), p.trackMaxServoSpeed, dtSec);
            txPrev = txF;
            txF = ema(txF, hasVision ? txDeg : txF, p.txAlpha);
            return servoCmd;
        }

        if (mode == Mode.LOCKED) {
            if (hasVision) {
                txF = ema(txF, txDeg, p.txAlpha);
                if (Math.abs(txF) > p.txTolDeg * 2.0) {
                    mode = Mode.AIMING;
                    lockCount = 0;
                }
            }
            return servoCmd;
        }

        double dx = targetX - x;
        double dy = targetY - y;
        double bearingRad = Math.atan2(dy, dx);
        double yawDesRad = wrapRad(bearingRad - (headingRad + headingBiasRad));

        double yawDesDeg = Math.toDegrees(yawDesRad);

        double servoFF = yawToServo(yawDesDeg);
        double sideBias = (yawDesDeg >= 0.0) ? rightBias : leftBias;
        servoFF = clamp(servoFF + centerBias + sideBias, p.servoMinLimit, p.servoMaxLimit);

        double maxSpeed = hasVision ? p.trackMaxServoSpeed : p.snapMaxServoSpeed;

        double servoTarget = servoFF;

        if (hasVision) {
            txF = ema(txF, txDeg, p.txAlpha);

            double txPred = txF + omegaDegPerSec * p.visionLatencySec;
            txPred = clamp(txPred, -45.0, 45.0);

            double dTx = (txPred - txPrev) / dtSec;
            txPrev = txPred;

            double txUsed = applyHysteresis(txPred, p.txTolDeg, p.txTolDeg * 1.8);

            double corr = p.kP * txUsed + p.kD * dTx;
            corr = clamp(corr, -p.corrMax, p.corrMax);

            servoTarget = clamp(servoFF + corr, p.servoMinLimit, p.servoMaxLimit);

            updateBiasLearning(yawDesDeg, txPred, omegaDegPerSec, servoTarget, dtSec);
            updateLockState(txPred);
        } else {
            txPrev = txF;
            lockCount = 0;
        }

        servoCmd = slewTo(servoCmd, servoTarget, maxSpeed, dtSec);
        servoCmd = clamp(servoCmd, p.servoMinLimit, p.servoMaxLimit);

        return servoCmd;
    }

    private void updateLockState(double txDeg) {
        if (Math.abs(txDeg) <= p.txTolDeg) {
            lockCount++;
            if (lockCount >= p.lockFrames) {
                mode = Mode.LOCKED;
            }
        } else {
            lockCount = 0;
        }
    }

    private void updateBiasLearning(double yawDesDeg, double txDeg, double omegaDegPerSec, double servoTarget, double dtSec) {
        if (p.enableBiasLearning <= 0.0) return;

        if (Math.abs(txDeg) > p.biasUpdateTxWindowDeg) return;
        if (Math.abs(omegaDegPerSec) > p.biasUpdateOmegaMaxDegPerSec) return;

        if (servoTarget <= p.servoMinLimit + p.biasUpdateLimitMargin) return;
        if (servoTarget >= p.servoMaxLimit - p.biasUpdateLimitMargin) return;

        double stepHeadDeg = clamp(p.headingBiasGain * txDeg, -p.biasUpdateMaxStepDeg, p.biasUpdateMaxStepDeg);
        headingBiasRad = clampRad(headingBiasRad + Math.toRadians(stepHeadDeg), Math.toRadians(p.headingBiasMaxDeg));

        double stepCenter = clamp(p.centerBiasGain * txDeg, -p.biasUpdateMaxStepServo, p.biasUpdateMaxStepServo);
        centerBias = clamp(centerBias + stepCenter, -p.centerBiasMax, p.centerBiasMax);

        double sideStep = clamp(0.8 * p.centerBiasGain * txDeg, -p.biasUpdateMaxStepServo, p.biasUpdateMaxStepServo);
        if (yawDesDeg >= 0.0) rightBias = clamp(rightBias + sideStep, -p.centerBiasMax, p.centerBiasMax);
        else leftBias = clamp(leftBias + sideStep, -p.centerBiasMax, p.centerBiasMax);
    }

    private double yawToServo(double yawDeg) {
        double y = clamp(yawDeg, -p.maxYawDeg, p.maxYawDeg);

        double base;
        if (y >= 0.0) {
            double slope = (p.servoRight - p.servoCenter) / p.maxYawDeg;
            base = p.servoCenter + slope * y;
        } else {
            double slope = (p.servoCenter - p.servoLeft) / p.maxYawDeg;
            base = p.servoCenter + slope * y;
        }
        return clamp(base, p.servoMinLimit, p.servoMaxLimit);
    }

    private void autoTuneGains() {
        double range = Math.max(1e-6, Math.abs(p.servoRight - p.servoLeft));
        double servoPerDeg = range / (2.0 * Math.max(1e-6, p.maxYawDeg));

        if (p.kP < 0) {
            p.kP = -0.85 * servoPerDeg;
        }
        if (p.kD < 0) {
            p.kD = -0.10 * servoPerDeg;
        }
    }

    private static double ema(double prev, double x, double alpha) {
        double a = clamp(alpha, 0.0, 1.0);
        return prev + a * (x - prev);
    }

    private static double applyHysteresis(double v, double enter, double exit) {
        double av = Math.abs(v);
        if (av < enter) return 0.0;
        if (av < exit) return v * 0.35;
        return v;
    }

    private static double slewTo(double current, double target, double maxSpeedPerSec, double dt) {
        double maxStep = Math.abs(maxSpeedPerSec) * dt;
        double d = target - current;
        if (d > maxStep) return current + maxStep;
        if (d < -maxStep) return current - maxStep;
        return target;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double wrapDeg(double a) {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }

    private static double clampRad(double v, double maxAbs) {
        if (v > maxAbs) return maxAbs;
        if (v < -maxAbs) return -maxAbs;
        return v;
    }
}