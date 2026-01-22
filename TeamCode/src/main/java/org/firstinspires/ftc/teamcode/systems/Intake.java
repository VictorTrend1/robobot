package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor motor;
    private Ruleta ruleta;

    private int balls = 0;
    private boolean full = false;

    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotor.class, "motor_intake");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setRuleta(Ruleta r) {
        ruleta = r;
    }

    public void resetForIntake() {
        balls = 0;
        full = false;
        stop();
    }

    public void start() { motor.setPower(1); }
    public void stop() { motor.setPower(0); }
    public void reverse() { motor.setPower(-1); }

    public boolean isReadyForScore() { return full; }
    public int getBallsIntaked() { return balls; }

    public Ruleta.Slot onBallEntered(boolean isGreen) {
        if (full || ruleta == null) return null;
        Ruleta.Slot placed = ruleta.onBallIntake(isGreen);
        if (placed == null) {
            full = true;
            stop();
            return null;
        }
        balls++;
        if (balls >= 3) {
            full = true;
            stop();
        }
        return placed;
    }
}
