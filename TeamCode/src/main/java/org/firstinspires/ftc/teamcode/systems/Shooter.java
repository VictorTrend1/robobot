package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    public final DcMotorEx flywheel;
    public final DcMotorEx flywheel2;
    private final Servo kicker;
    // public final DcMotorEx kicker;

    private  double TARGET_VEL = 1700;
    private  double MIN_VEL = 1680;
    private double KICK_PUSH = 0.14;
    private double KICK_RETRACT = 0.32;

    private static final PIDFCoefficients PIDF =
            new PIDFCoefficients(300, 0, 1, 12);

    public Shooter(HardwareMap hw) {
        flywheel = hw.get(DcMotorEx.class, "shooter1");
        flywheel2 = hw.get(DcMotorEx.class, "shooter2");
        /// TODO reverse la un motor !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        kicker = hw.get(Servo.class, "kicker");
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
        flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
        stopAll();
    }
    public void spinUpTo(int target_vel){flywheel.setVelocity(target_vel);
        flywheel2.setPower(flywheel.getPower());
    }
    public boolean atSpeedTo(int min_vel){
        return flywheel.getVelocity() >= min_vel;
    }
    public void spinUp() { flywheel.setVelocity(TARGET_VEL);
        flywheel2.setPower(flywheel.getPower()); }
    public void stopFlywheel() { flywheel.setVelocity(0);flywheel2.setVelocity(0); }

    public boolean atSpeed() { return flywheel.getVelocity() >= MIN_VEL; }
    public double getVelocity() { return flywheel.getVelocity(); }

    public void pushKicker() { kicker.setPosition(KICK_PUSH); }
    public void retractKicker() { kicker.setPosition(KICK_RETRACT); }

    public void toggleRPM() {
        if (TARGET_VEL == 1700) {
            TARGET_VEL = 1400;
            MIN_VEL = 1350;
        } else {
            TARGET_VEL = 1700;
            MIN_VEL = 1650;
        }
    }
    public String showRpm() {
        if (TARGET_VEL == 1700) {
            return "FAR";
        } else if(TARGET_VEL == 1400 ){
            return "CLOSE";
        }else{
            return "NULL";
        }
    }

    public void stopAll() {
        stopFlywheel();
       retractKicker();
    }

}
