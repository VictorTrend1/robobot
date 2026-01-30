package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.systems.Tureta;
@Config
public class autoAim_LL {

    private static final int Tx_SIGN = -1;



    private final Tureta tureta;
    public Limelight3A limelight;
    private boolean started = false;

    private static final double DEADBAND = 4;     // deg

    private static final double K = 0.2;         // servoUnitsPerDegPerSec
    private static final double MAX_VEL = 0.12;     // servoUnitsPerSec (limits speed)

    private long lastNs = 0;

    private boolean dirReady = false;
    private int dirSign = +1;
    private boolean probePending = false;
    private double probeBasePos = 0.0;
    private double probeBaseAbsTx = 0.0;
    private long lastValidMs = 0L;
    private static final long HOLD_MS = 200;
    private double txFilt = 0.0;
    private static final double TX_ALPHA = 0.3;
    private static final double PROBE_DELTA = 0.006;
    private static final double K_VEL = 0.06;

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
        txFilt = 0.0;
        lastNs = 0L;
        lastValidMs = 0L;
        dirReady = false;
        dirSign = +1;
        probePending = false;
        probeBasePos = 0.0;
        probeBaseAbsTx = 0.0;
    }

    public void updateAim() {
        if (!started || limelight == null) return;

        LLResult r = limelight.getLatestResult();
        long nowMs = System.currentTimeMillis();

        if (r == null || !r.isValid()) {
            if (lastValidMs != 0 && (nowMs - lastValidMs) > HOLD_MS) {
                tureta.goDefault();
                dirReady = false;
                probePending = false;
                lastNs = 0L;
            }
            return;
        }

        double tx = r.getTx();
        lastValidMs = nowMs;

        txFilt = txFilt + TX_ALPHA * (tx - txFilt);

        if (!dirReady) {
            if (!probePending) {
                probeBaseAbsTx = Math.abs(txFilt);
                probeBasePos = tureta.getPosition();
                tureta.setPosition(probeBasePos + PROBE_DELTA);
                probePending = true;

            } else {
                double newAbs = Math.abs(txFilt);
                if (newAbs < probeBaseAbsTx) dirSign = +1;
                else dirSign = -1;
                tureta.setPosition(probeBasePos);
                probePending = false;
                dirReady = true;

            }
        }

        long nowNs = System.nanoTime();
        double dt = (lastNs == 0L) ? 0.02 : (nowNs - lastNs) * 1e-9;
        lastNs = nowNs;
        if (dt < 0.008) dt = 0.008;
        if (dt > 0.050) dt = 0.050;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            if (Math.abs(tx) > DEADBAND) {
                double vel = Tx_SIGN * K * tx;
                if (vel > MAX_VEL) vel = MAX_VEL;
                if (vel < -MAX_VEL) vel = -MAX_VEL;

                double next = tureta.getPosition() + vel * dt;
                tureta.setPosition(Tureta.clamp(next));
            }
        }
    }

    public  void getToPos(boolean pressed){
        if(pressed){
            double sum = 0;
            int n = 0;
            for (int i = 0; i < 3; i++) {
                LLResult r = limelight.getLatestResult();
                if (r != null && r.isValid()) {
                    sum += r.getTx();
                    n++;
                }
                try { Thread.sleep(20); } catch (InterruptedException ignored) {}
            }

            if (n > 0) {
                double tx = sum / n;

                double POS_PER_DEG = 0.003;
                int TX_SIGN = -1;

                double target = tureta.getPosition() + TX_SIGN * tx * POS_PER_DEG;
                tureta.setPosition(target);
            }
        }
    }
}
