package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.Tureta;

public class autoAim_LL {
    private static final int PIPELINE_INDEX = 2;

    private static final double POS_MIN = 0.20;
    private static final double POS_MAX = 0.80;

    private static final double KP = 0.010;
    private static final double KD = 0.002;
    private static final double DEADBAND_DEG = 0.50;
    private static final double MAX_STEP = 0.020;
    private static final int ty_SIGN = +1;

    private static final int STABLE_FRAMES_REQUIRED = 2;
    private static final long HOLD_LAST_MS = 100;
    private static final double ty_ALPHA = 0.35;
    private static final double MAX_JUMP_DEG = 10.0;

    private Tureta tureta;
    private Limelight3A limelight;
    private boolean started = false;

    private int stableCount = 0;
    private long lastGoodMs = 0;

    private boolean filtInit = false;
    private double tyFilt = 0.0;
    private double tyFiltPrev = 0.0;

    private long lastNs = 0L;

    public autoAim_LL(Tureta tureta) {
        this.tureta = tureta;
    }

    public void init(HardwareMap hw, int index) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(index);
    }

    public void start() {
        if (limelight == null) return;
        limelight.start();
        started = true;
        reset();
    }

    public void reset() {
        stableCount = 0;
        lastGoodMs = 0;
        filtInit = false;
        tyFilt = 0.0;
        tyFiltPrev = 0.0;
        lastNs = 0L;
    }

    public boolean updateAim() {
        if (!started || limelight == null) return false;

        LLResult result = limelight.getLatestResult();
        long nowMs = System.currentTimeMillis();

        if (result != null && result.isValid()) {
            double ty = result.getTy();

            if (!filtInit) {
                tyFilt = ty;
                tyFiltPrev = ty;
                filtInit = true;
                lastNs = System.nanoTime();
                stableCount = 1;
                lastGoodMs = nowMs;
                return false;
            }

            if (Math.abs(ty - tyFilt) > MAX_JUMP_DEG) {
                stableCount = 0;
                return false;
            }

            tyFiltPrev = tyFilt;
            tyFilt = tyFilt + ty_ALPHA * (ty - tyFilt);

            lastGoodMs = nowMs;
            stableCount++;

        } else {
            if (!(filtInit && (nowMs - lastGoodMs) <= HOLD_LAST_MS)) {
                stableCount = 0;
                filtInit = false;
                tureta.goDefault();
                return false;
            }
            stableCount = Math.min(stableCount, STABLE_FRAMES_REQUIRED);
        }


        if (stableCount < STABLE_FRAMES_REQUIRED) return false;

        long nowNs = System.nanoTime();
        double dt = (lastNs == 0L) ? 0.02 : (nowNs - lastNs) * 1e-9;
        lastNs = nowNs;
        if (dt < 0.008) dt = 0.008;
        if (dt > 0.120) dt = 0.120;

        double errDeg = ty_SIGN * tyFilt;
        if (Math.abs(errDeg) <= DEADBAND_DEG) return false;

        double dErr = (tyFilt - tyFiltPrev) / dt;

        double delta = (KP * errDeg) + (KD * dErr);
        if (delta > MAX_STEP) delta = MAX_STEP;
        if (delta < -MAX_STEP) delta = -MAX_STEP;

        double next = tureta.getPosition() + delta;
        if (next < POS_MIN) next = POS_MIN;
        if (next > POS_MAX) next = POS_MAX;

        tureta.setPosition(next);
        return true;
    }
}