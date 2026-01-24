package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.systems.Tureta;

@Config
public class aim {

    public static double TARGET_X = 0.0;
    public static double TARGET_Y = 0.0;

    public static double SERVO_CENTER = 0.47;
    public static double SERVO_RANGE_DEG = 220.0;
    public static double POS_RIGHT_LIMIT = 0.25;
    public static double POS_LEFT_LIMIT  = 0.80;
    public static boolean INVERT_TURRET = false;

    public static double AIM_OFFSET_LEFT_DEG = 0.0;
    public static double AIM_OFFSET_RIGHT_DEG = 0.0;

    public static double DEAD_ENTER_DEG = 2.0;
    public static double DEAD_EXIT_DEG  = 4.0;

    public static double DIST_HOLD = 55.0;
    public static double MAX_BEARING_RATE_DEG_PER_SEC = 120.0;

    public static double POSE_ALPHA = 0.12;
    public static double ERR_ALPHA  = 0.18;

    public static double MAX_SERVO_VEL   = 0.30;
    public static double MAX_SERVO_ACCEL = 1.0;

    public static double LIMIT_SLOW_ZONE = 0.05;
    public static double LIMIT_MIN_VEL_SCALE = 0.20;
    public static double LIMIT_MIN_ACCEL_SCALE = 0.25;

    public static double STICKY_MARGIN = 0.004;

    private final Tureta tureta;

    private boolean enabled = true;

    private boolean filtInit = false;
    private double xF = 0, yF = 0;
    private double rF = 0;
    private double rUnwrap = 0;
    private double lastRMeas = 0;

    private boolean bearingInit = false;
    private double bearingHold = 0.0;

    private boolean hold = false;
    private double errFState = 0.0;
    private double servoVel = 0.0;

    private double lastDist = 0.0;
    private double lastErrDeg = 0.0;
    private double lastResidualDeg = 0.0;
    private boolean lastSaturated = false;

    public aim(Tureta tureta) {
        this.tureta = tureta;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            servoVel = 0.0;
            hold = true;
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void reset(Pose2d pose) {
        filtInit = false;
        bearingInit = false;
        hold = false;
        errFState = 0.0;
        servoVel = 0.0;
        update(pose, 0.02);
    }

    public void update(Pose2d pose, double dt) {
        if (!enabled) return;

        double lx = pose.position.x;
        double ly = pose.position.y;
        double rMeas = angleWrap(pose.heading.toDouble());

        if (!filtInit) {
            xF = lx;
            yF = ly;
            lastRMeas = rMeas;
            rUnwrap = rMeas;
            rF = rMeas;
            errFState = 0.0;
            servoVel = 0.0;
            filtInit = true;
        } else {
            double dR = angleWrap(rMeas - lastRMeas);
            rUnwrap += dR;
            lastRMeas = rMeas;

            xF = (1.0 - POSE_ALPHA) * xF + POSE_ALPHA * lx;
            yF = (1.0 - POSE_ALPHA) * yF + POSE_ALPHA * ly;

            double rUnwrapF = unwrapFilterStep(rF, rUnwrap, POSE_ALPHA);
            rF = angleWrap(rUnwrapF);
        }

        double x = xF - TARGET_X;
        double y = yF - TARGET_Y;
        double r = angleWrap(rF);

        double dx = -x;
        double dy = -y;
        double dist = Math.hypot(dx, dy);
        lastDist = dist;

        double rangeRad = Math.toRadians(SERVO_RANGE_DEG);
        double maxLeftRad  = (POS_LEFT_LIMIT  - SERVO_CENTER) * rangeRad;
        double maxRightRad = (POS_RIGHT_LIMIT - SERVO_CENTER) * rangeRad;

        if (!bearingInit) {
            bearingHold = Math.atan2(dy, dx);
            bearingInit = true;
        }

        double bearing;
        if (dist <= DIST_HOLD) {
            bearing = bearingHold;
            hold = true;
        } else {
            double bearingRaw = Math.atan2(dy, dx);
            double maxStep = Math.toRadians(MAX_BEARING_RATE_DEG_PER_SEC) * dt;
            bearingHold = angleApproach(bearingHold, bearingRaw, maxStep);
            bearing = bearingHold;
        }

        double err = angleWrap(bearing - r);
        double sign = INVERT_TURRET ? -1.0 : 1.0;
        double errTurret = angleWrap(sign * err);

        double offDeg = (errTurret >= 0.0) ? AIM_OFFSET_LEFT_DEG : AIM_OFFSET_RIGHT_DEG;
        errTurret = angleWrap(errTurret + Math.toRadians(offDeg));

        double errSat = clamp(errTurret, maxRightRad, maxLeftRad);

        errFState = (1.0 - ERR_ALPHA) * errFState + ERR_ALPHA * errSat;
        errFState = angleWrap(errFState);

        lastErrDeg = Math.toDegrees(errFState);

        double absErrDeg = Math.abs(lastErrDeg);
        if (!hold && absErrDeg <= DEAD_ENTER_DEG) hold = true;
        if (hold && absErrDeg >= DEAD_EXIT_DEG) hold = false;

        double currentServo = tureta.getPosition();
        double targetServo = currentServo;

        lastSaturated = false;

        if (!hold) {
            double servoRaw = servoFromTurretAngle(errFState, rangeRad);
            double servoCmd = clamp(servoRaw, POS_RIGHT_LIMIT, POS_LEFT_LIMIT);

            boolean saturated = Math.abs(servoRaw - servoCmd) > 1e-4;
            lastSaturated = saturated;

            if (saturated) {
                errFState = errSat;
                servoVel = approach(servoVel, 0.0, MAX_SERVO_ACCEL * dt);
                targetServo = servoCmd;
            } else {
                targetServo = applyStickyLimits(servoCmd, currentServo);
            }
        } else {
            targetServo = currentServo;
        }

        double velScale = limitScale(currentServo, LIMIT_SLOW_ZONE, LIMIT_MIN_VEL_SCALE);
        double accelScale = limitScale(currentServo, LIMIT_SLOW_ZONE, LIMIT_MIN_ACCEL_SCALE);

        double vMax = MAX_SERVO_VEL * velScale;
        double aMax = MAX_SERVO_ACCEL * accelScale;

        if (Math.abs(targetServo - currentServo) < 1e-6) {
            servoVel = approach(servoVel, 0.0, aMax * dt);
        } else {
            double desiredVel = clamp((targetServo - currentServo) / dt, -vMax, vMax);
            servoVel = approach(servoVel, desiredVel, aMax * dt);
        }

        double nextServo = currentServo + servoVel * dt;
        nextServo = clamp(nextServo, POS_RIGHT_LIMIT, POS_LEFT_LIMIT);
        nextServo = applyStickyLimits(nextServo, currentServo);

        tureta.setPosition(nextServo);

        double turretAngleRad = (tureta.getPosition() - SERVO_CENTER) * rangeRad;
        double residualRad = angleWrap(errTurret - turretAngleRad);
        lastResidualDeg = Math.toDegrees(residualRad);
    }

    public double getDistance() {
        return lastDist;
    }

    public double getErrDeg() {
        return lastErrDeg;
    }

    public double getResidualDeg() {
        return lastResidualDeg;
    }

    public boolean isHold() {
        return hold;
    }

    public boolean isSaturated() {
        return lastSaturated;
    }

    public double getTurretServo() {
        return tureta.getPosition();
    }

    private double servoFromTurretAngle(double turretAngleRad, double rangeRad) {
        double a = turretAngleRad;

        if (a >= 0.0) {
            double maxAng = (POS_LEFT_LIMIT - SERVO_CENTER) * rangeRad;
            if (maxAng <= 1e-9) return SERVO_CENTER;
            double t = a / maxAng;
            return SERVO_CENTER + (POS_LEFT_LIMIT - SERVO_CENTER) * t;
        } else {
            double maxAng = (SERVO_CENTER - POS_RIGHT_LIMIT) * rangeRad;
            if (maxAng <= 1e-9) return SERVO_CENTER;
            double t = (-a) / maxAng;
            return SERVO_CENTER - (SERVO_CENTER - POS_RIGHT_LIMIT) * t;
        }
    }

    private double limitScale(double servoPos, double slowZone, double minScale) {
        double dRight = servoPos - POS_RIGHT_LIMIT;
        double dLeft = POS_LEFT_LIMIT - servoPos;
        double d = Math.min(dRight, dLeft);
        if (d >= slowZone) return 1.0;
        if (d <= 0.0) return minScale;
        double u = d / slowZone;
        double s = u * u * (3.0 - 2.0 * u);
        return minScale + (1.0 - minScale) * s;
    }

    private double applyStickyLimits(double candidate, double current) {
        if (current <= POS_RIGHT_LIMIT + STICKY_MARGIN) {
            if (candidate <= current) return POS_RIGHT_LIMIT;
        }
        if (current >= POS_LEFT_LIMIT - STICKY_MARGIN) {
            if (candidate >= current) return POS_LEFT_LIMIT;
        }
        return candidate;
    }

    private static double angleWrap(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double angleApproach(double current, double target, double maxStep) {
        double delta = angleWrap(target - current);
        if (delta > maxStep) delta = maxStep;
        if (delta < -maxStep) delta = -maxStep;
        return angleWrap(current + delta);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double unwrapFilterStep(double currentWrapped, double targetUnwrapped, double alpha) {
        double cw = currentWrapped;
        while (cw - targetUnwrapped > Math.PI) cw -= 2.0 * Math.PI;
        while (cw - targetUnwrapped < -Math.PI) cw += 2.0 * Math.PI;
        return (1.0 - alpha) * cw + alpha * targetUnwrapped;
    }

    private static double approach(double current, double target, double maxDelta) {
        double delta = target - current;
        if (delta > maxDelta) return current + maxDelta;
        if (delta < -maxDelta) return current - maxDelta;
        return target;
    }

    public double turretDegreesToServo(double turretDeg) {
        double turretRad = Math.toRadians(turretDeg);
        double rangeRad = Math.toRadians(SERVO_RANGE_DEG);

        double maxLeftRad  = (POS_LEFT_LIMIT  - SERVO_CENTER) * rangeRad;
        double maxRightRad = (POS_RIGHT_LIMIT - SERVO_CENTER) * rangeRad;

        double servo;

        if (turretRad >= 0.0) {
            if (maxLeftRad <= 1e-9) return SERVO_CENTER;
            double t = turretRad / maxLeftRad;
            servo = SERVO_CENTER + (POS_LEFT_LIMIT - SERVO_CENTER) * t;
        } else {
            if (maxRightRad >= -1e-9) return SERVO_CENTER;
            double t = turretRad / maxRightRad;
            servo = SERVO_CENTER + (POS_RIGHT_LIMIT - SERVO_CENTER) * t;
        }

        return clamp(servo, POS_RIGHT_LIMIT, POS_LEFT_LIMIT);
    }

}
