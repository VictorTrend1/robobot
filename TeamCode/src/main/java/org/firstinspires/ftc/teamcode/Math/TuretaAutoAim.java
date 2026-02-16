package org.firstinspires.ftc.teamcode.Math;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Tureta;

public class TuretaAutoAim {

    private final Tureta tureta;

    private double targetX = 0.0;
    private double targetY = 0.0;

    private static final double SERVO_POS_MIN = 0.32;
    private static final double SERVO_POS_MAX = 0.66;
    private static final double SERVO_POS_CENTRU = 0.5;

    private static final double SERVO_ZERO = SERVO_POS_CENTRU;

    private static final double SERVO_RANGE_DEGREES = 115.0;

    private PinpointDrive drive;

    public TuretaAutoAim(HardwareMap hardwareMap, PinpointDrive drive) {
        this.tureta = new Tureta(hardwareMap);
        this.drive = drive;
    }

    public void setTarget(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    private double servoPositionToAngle(double servoPos) {
        double servoRange = SERVO_POS_MAX - SERVO_POS_MIN;
        double positionOffset = servoPos - SERVO_ZERO;
        double normalizedOffset = positionOffset / servoRange;
        return normalizedOffset * SERVO_RANGE_DEGREES;
    }


    private double angleToServoPosition(double angleDegrees) {
        double servoRange = SERVO_POS_MAX - SERVO_POS_MIN;
        double normalizedAngle = angleDegrees / SERVO_RANGE_DEGREES;
        double servoPos = SERVO_ZERO + (normalizedAngle * servoRange);


        return Math.max(SERVO_POS_MIN, Math.min(SERVO_POS_MAX, servoPos));
    }


    public double getCurrentAngle() {
        return servoPositionToAngle(tureta.getPosition());
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


    public boolean aimToTarget() {

        double robotX = drive.pose.position.x;
        double robotY = drive.pose.position.y;


        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double destinationAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));


        double robotHeading = getRobotHeading();


        double requiredTurretAngle = normalizeAngle(destinationAngle - robotHeading);


        double targetServoPos = angleToServoPosition(requiredTurretAngle);


        if (!isServoPositionValid(targetServoPos)) {

            targetServoPos = Tureta.clamp(targetServoPos);
            tureta.setPosition(targetServoPos);
            return false;
        }


        tureta.setPosition(targetServoPos);
        return true;
    }


    public boolean isAimed(double toleranceDegrees) {
        double robotX = drive.pose.position.x;
        double robotY = drive.pose.position.y;

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double destinationAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        double robotHeading = getRobotHeading();
        double currentTurretAngle = getCurrentAngle();


        double currentAimAngle = normalizeAngle(robotHeading + currentTurretAngle);


        double error = Math.abs(normalizeAngle(destinationAngle - currentAimAngle));
        return error < toleranceDegrees;
    }

    public double getDistance() {
        double robotX = drive.pose.position.x;
        double robotY = drive.pose.position.y;
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public Tureta getTureta() {
        return tureta;
    }


}