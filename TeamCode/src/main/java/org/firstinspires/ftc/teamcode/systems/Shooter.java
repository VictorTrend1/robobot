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

    private  double TARGET_VEL = 1700;
    private  double MIN_VEL = 1680;
    private double KICK_PUSH = 0.14;
    private double KICK_RETRACT = 0.32;
    private static final double POZ_X = -75.0;


    private static final PIDFCoefficients PIDF =
            new PIDFCoefficients(300, 0, 1, 12);

    public Shooter(HardwareMap hw) {
        flywheel = hw.get(DcMotorEx.class, "shooter1");
        flywheel2 = hw.get(DcMotorEx.class, "shooter2");
        kicker = hw.get(Servo.class, "kicker");
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
        flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
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
    public void setRPMForDistance(double distanceInches) {
        double rpm = trajectory_Interpolation.rpmForDistance(distanceInches);
        flywheel.setVelocity(rpm);
        flywheel2.setPower(flywheel.getPower());
    }

}