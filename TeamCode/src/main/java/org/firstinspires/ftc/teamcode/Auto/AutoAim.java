package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Math.TargetTracker;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Tureta;

public abstract class AutoAim {

    protected Limelight3A limelight;
    protected TargetTracker aimer;
    protected Tureta tureta;
    protected PinpointDrive drive;
    protected long lastNs;


    protected int pipeline = 2;

    protected double targetX = 0.0;
    protected double targetY = 0.0;


    public final void init(HardwareMap hardwareMap, PinpointDrive drive) {
        this.drive = drive;


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.start();

        tureta = new Tureta(hardwareMap);

        TargetTracker.Params ap = getAimerParams();
        ap.targetX = targetX;
        ap.targetY = targetY;

        aimer = new TargetTracker(ap);
        tureta.setPosition(ap.servoCenter);

        lastNs = System.nanoTime();

        onInit();
    }

    protected void onInit() {
    }


    protected TargetTracker.Params getAimerParams() {
        TargetTracker.Params ap = new TargetTracker.Params();
        ap.servoCenter = 0.5;
        ap.servoLeft = 0.3;
        ap.servoRight = 0.7;
        ap.servoMinLimit = 0.3;
        ap.servoMaxLimit = 0.7;
        ap.maxYawDeg = 110.0;
        return ap;
    }


    public final void setTarget(double x, double y) {
        aimer.setTarget(x, y);
    }


    public final double getTargetX() {
        return aimer.getTargetX();
    }


    public final double getTargetY() {
        return aimer.getTargetY();
    }


    public final void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }


    public final void startAiming() {
        aimer.requestAim();
    }


    public final boolean updateAim() {
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastNs) * 1e-9;
        lastNs = nowNs;
        if (dt < 0.008) dt = 0.008;
        if (dt > 0.050) dt = 0.050;

        drive.updatePoseEstimate();
        Pose2d pose = drive.pose;

        LLResult r = limelight.getLatestResult();
        boolean hasVision = (r != null) && r.isValid();
        double tx = hasVision ? r.getTx() : 0.0;

        double servoCmd = aimer.update(
                pose.position.x,
                pose.position.y,
                pose.heading.toDouble(),
                hasVision,
                tx,
                dt
        );

        tureta.setPosition(servoCmd);

        return aimer.isLocked();
    }


    public final boolean aimAndWait(long timeoutMs) {
        startAiming();
        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < timeoutMs) {
            if (updateAim()) {
                return true;
            }
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return false;
            }
        }

        return false;
    }


    public final boolean aimAtAndWait(double x, double y, long timeoutMs) {
        setTarget(x, y);
        return aimAndWait(timeoutMs);
    }


    public final void cancel() {
        aimer.cancel();
        tureta.goDefault();
    }

    public final void reset() {
        aimer.reset();
        tureta.setPosition(0.5);
    }


    public final boolean isLocked() {
        return aimer.isLocked();
    }

    public final TargetTracker.Mode getMode() {
        return aimer.getMode();
    }

    public final boolean hasVision() {
        LLResult r = limelight.getLatestResult();
        return (r != null) && r.isValid();
    }

    public final void setTuretaPosition(double position) {
        tureta.setPosition(position);
    }
}