package org.firstinspires.ftc.teamcode.Math;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Tureta;

@Config
public class TuretaAutoAim {

    private final Tureta tureta;
    private PinpointDrive drive;

    private double targetX = 0.0;
    private double targetY = 0.0;

    private static final double SERVO_POS_MIN = 0.15;
    private static final double SERVO_POS_MAX = 0.85;
    private static final double SERVO_CENTER = 0.50;
    public static double GAIN = 1;
    public static double PINPOINT_TOLERANCE_DEG = 40.0;
    public static double LIMELIGHT_GAIN = -1.1;
    public static double LIMELIGHT_WEIGHT = 0.8;
    private double limelightTx = 0.0;
    private Limelight3A limelight;



    private static final double SERVO_RANGE_DEGREES = 170;

    private PoseVelocity2d currentVelocity = new PoseVelocity2d(
            new Vector2d(0, 0), 0);

    public TuretaAutoAim(HardwareMap hardwareMap, PinpointDrive drive) {
        this.tureta = new Tureta(hardwareMap);
        this.drive = drive;
    }

    public void setTarget(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void setVelocity(PoseVelocity2d velocity) {
        if (velocity != null) {
            this.currentVelocity = velocity;
        }
    }
    public void setLimelightTx(Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            limelightTx = result.getTx();
        } else {
            limelightTx = 0.0;
        }
    }
    public double returnTx(){
        return limelightTx;
    }
    private double angleToServoPosition(double angleDegrees) {
        double servoRange = SERVO_POS_MAX - SERVO_POS_MIN;
        double normalizedAngle = angleDegrees / SERVO_RANGE_DEGREES;
        double servoPos = SERVO_CENTER + (normalizedAngle * servoRange);
        return Math.max(SERVO_POS_MIN, Math.min(SERVO_POS_MAX, servoPos));
    }

    private double applyGain(double requiredTurretAngle) {
        return requiredTurretAngle * GAIN;

    }

    public boolean aimToTarget(double TOLERANCE) {
        double robotX = drive.pose.position.x;
        double robotY = drive.pose.position.y;

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;

        if (Math.sqrt(deltaX * deltaX + deltaY * deltaY) < 0.001) {
            return false;
        }

        double destinationAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotHeading = getRobotHeading();
        double requiredTurretAngle = normalizeAngle(destinationAngle - robotHeading);
        double headingError = requiredTurretAngle;

        if (Math.abs(headingError) > PINPOINT_TOLERANCE_DEG) {
            requiredTurretAngle = applyGain(requiredTurretAngle);
        }
        else {
            double limelightCorrection = limelightTx * LIMELIGHT_GAIN;
            requiredTurretAngle = requiredTurretAngle * (1.0 - LIMELIGHT_WEIGHT)
                    + (requiredTurretAngle + limelightCorrection) * LIMELIGHT_WEIGHT;
        }


        if (Math.abs(getHeadingError()) < TOLERANCE ){
            tureta.setPosition(angleToServoPosition(requiredTurretAngle));
            return true;
        }

        double targetServoPos = angleToServoPosition(requiredTurretAngle);

        if (Double.isNaN(targetServoPos) || Double.isInfinite(targetServoPos)) {
            return false;
        }

        if (!isServoPositionValid(targetServoPos)) {
            targetServoPos = Tureta.clamp(targetServoPos);
            tureta.setPosition(targetServoPos);
            return false;
        }

        tureta.setPosition(targetServoPos);
        return true;
    }


    public boolean isAimed(double toleranceDegrees) {
        return Math.abs(getHeadingError()) < toleranceDegrees;
    }

    public double getHeadingError() {
        double robotX = drive.pose.position.x;
        double robotY = drive.pose.position.y;

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;

        double destinationAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotHeading = getRobotHeading();

        return normalizeAngle(destinationAngle - robotHeading);
    }

    public double getDistance() {
        double robotX = drive.pose.position.x;
        double robotY = drive.pose.position.y;
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    private double getRobotHeading() {
        return Math.toDegrees(drive.pose.heading.toDouble());
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private boolean isServoPositionValid(double servoPos) {
        return servoPos >= SERVO_POS_MIN && servoPos <= SERVO_POS_MAX;
    }

    public Tureta getTureta() {
        return tureta;
    }


}