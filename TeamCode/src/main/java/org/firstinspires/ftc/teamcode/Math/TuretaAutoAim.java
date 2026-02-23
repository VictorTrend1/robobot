package org.firstinspires.ftc.teamcode.Math;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
    public static double LEFT_GAIN = 1.22;
    public static double RIGHT_GAIN = 1.22;


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

    private double angleToServoPosition(double angleDegrees) {
        double servoRange = SERVO_POS_MAX - SERVO_POS_MIN;
        double normalizedAngle = angleDegrees / SERVO_RANGE_DEGREES;
        double servoPos = SERVO_CENTER + (normalizedAngle * servoRange);
        return Math.max(SERVO_POS_MIN, Math.min(SERVO_POS_MAX, servoPos));
    }

    private double applyGain(double requiredTurretAngle) {
        if (requiredTurretAngle < 0) {
            return requiredTurretAngle * RIGHT_GAIN;
        } else {
            return requiredTurretAngle * LEFT_GAIN;
        }
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

        requiredTurretAngle = applyGain(requiredTurretAngle);

        if (Math.abs(getHeadingError()) < Math.toRadians(TOLERANCE)) {
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
        double headingRad = drive.pose.heading.toDouble();
        while (headingRad > Math.PI) headingRad -= 2 * Math.PI;
        while (headingRad < -Math.PI) headingRad += 2 * Math.PI;
        return Math.toDegrees(headingRad);
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