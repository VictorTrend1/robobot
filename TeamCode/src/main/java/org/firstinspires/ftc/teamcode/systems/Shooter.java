package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.pow;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Math.trajectory_Interpolation;

public class Shooter {

    public final DcMotorEx flywheel;
    public final DcMotorEx flywheel2;
    private final Servo kicker;

    private  double TARGET_VEL = 1640;
    private  double MIN_VEL = 1620;
    private double KICK_PUSH = 0.17;
    private double KICK_RETRACT = 0.297;
    private static final double DISTANTA = 90.0;


    private static final PIDFCoefficients PIDF =
            new PIDFCoefficients(450, 0, 1, 11);

    public Shooter(HardwareMap hw) {
        flywheel = hw.get(DcMotorEx.class, "shooter1");
        flywheel2 = hw.get(DcMotorEx.class, "shooter2");
        kicker = hw.get(Servo.class, "kicker");
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopAll();
    }
    public void spinUpTo(double target_vel){flywheel.setVelocity(target_vel);
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


    public void stopAll() {
        stopFlywheel();
       retractKicker();
    }
    public void setKicker(double pos){kicker.setPosition(pos);}

    public void RPMPos(double distance) {
        if (distance > DISTANTA) {
            setRPMFar();
        } else {
            setRPMClose();
        }
    }
    public void setRPMFar() {
        if (TARGET_VEL != 1640) {
            TARGET_VEL = 1640;
            MIN_VEL = 1620;
            if (flywheel.getVelocity() > 0) {
                spinUp();
            }
        }
    }

    public void setRPMClose() {
        if (TARGET_VEL != 1450) {
            TARGET_VEL = 1450;
            MIN_VEL = 1400;
            if (flywheel.getVelocity() > 0) {
                spinUp();
            }
        }
    }

    public void setRPMForDistance(double distanceInches) {
        double rpm = trajectory_Interpolation.rpmForDistance(distanceInches);
        flywheel.setVelocity(rpm);
        flywheel2.setPower(flywheel.getPower());
    }

}