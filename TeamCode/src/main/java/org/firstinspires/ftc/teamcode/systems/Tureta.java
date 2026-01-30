package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tureta {

    private static final double POS_MIN = 0.10;
    private static final double POS_MAX = 0.50;
    private static final double POS_DEFAULT = 0.30;

    private final Servo s1;
    private final Servo s2;

    private double lastPos = POS_DEFAULT;

    public Tureta(HardwareMap hardwareMap) {
        s1 = hardwareMap.get(Servo.class, "tureta1");
        s2 = hardwareMap.get(Servo.class, "tureta2");
    }

    public void setPosition(double requestedPos) {
        double pos = requestedPos;
        lastPos = pos;
        s1.setPosition(pos);
        s2.setPosition(pos);
    }

    public void goDefault() {
        setPosition(POS_DEFAULT);
    }

    public double getPosition() {
        return lastPos;
    }

    public boolean isAtMin() {
        return lastPos <= POS_MIN;
    }

    public boolean isAtMax() {
        return lastPos >= POS_MAX;
    }

    public static double clamp(double v) {
        if (v <= POS_MIN) return POS_MIN+0.01;
        if (v >= POS_MAX) return POS_MAX-0.01;
        return v;
    }
}
