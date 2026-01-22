package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Tureta;

@TeleOp(name = "liveTracking")
public class liveTracking extends LinearOpMode {

    public static double TARGET_X = 0.0;
    public static double TARGET_Y = 0.0;

    public static double START_REL_TO_TARGET_X = 36.0;
    public static double START_REL_TO_TARGET_Y = 18.0;
    public static double START_REL_TO_TARGET_R_DEG = 0.0;

    public static double SERVO_CENTER = 0.50;
    public static double SERVO_RANGE_DEG = 220.0;
    public static double POS_RIGHT_LIMIT = 0.20;
    public static double POS_LEFT_LIMIT  = 0.80;
    public static boolean INVERT_TURRET = false;

    public static double DEAD_ENTER_DEG = 10.0;//TODO in caz ca e prea lent sau prea rapid
    public static double DEAD_EXIT_DEG  = 13.0;
    public static double DIST_HOLD = 2.0;
    public static double POSE_ALPHA = 0.18;//TODO scade sau creste filtrarea daca exista delay la tracking
    public static double ERR_ALPHA = 0.18;
    public static double MAX_SERVO_VEL = 0.45;
    public static double MAX_SERVO_ACCEL = 2.0;
    public static double STICKY_MARGIN = 0.02;
    public static double DT_MIN = 0.01;
    public static double DT_MAX = 0.05;

    //TODO baga un heading offset daca exista defazare la default angle

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PinpointDrive drive = new PinpointDrive(
                hardwareMap,
                new Pose2d(new Vector2d(0, 0), Math.toRadians(0))
        );

        Tureta tureta = new Tureta(hardwareMap);
        tureta.goDefault();

        final double startX = START_REL_TO_TARGET_X;
        final double startY = START_REL_TO_TARGET_Y;
        final double startR = Math.toRadians(START_REL_TO_TARGET_R_DEG);

        boolean filtInit = false;
        double xF = 0, yF = 0;
        double rF = 0;
        double rUnwrap = 0;
        double lastRMeas = 0;

        boolean hold = false;
        double errF = 0;
        double servoVel = 0;

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("StartRelTarget", "%.2f %.2f %.1f",
                    startX, startY, START_REL_TO_TARGET_R_DEG);
            telemetry.update();
        }

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive()) {

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            double dt = timer.seconds();
            timer.reset();
            dt = clamp(dt, DT_MIN, DT_MAX);

            double lx = drive.pose.position.x;
            double ly = drive.pose.position.y;
            double rMeas = angleWrap(drive.pose.heading.toDouble());

            if (!filtInit) {
                xF = lx;
                yF = ly;
                lastRMeas = rMeas;
                rUnwrap = rMeas;
                rF = rMeas;
                errF = 0;
                servoVel = 0;
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

            double x = startX + xF - TARGET_X;
            double y = startY + yF - TARGET_Y;
            double r = angleWrap(startR + rF);

            double dx = -x;
            double dy = -y;
            double dist = Math.hypot(dx, dy);

            double currentServo = tureta.getPosition();
            double targetServo = currentServo;
            double sign = INVERT_TURRET ? -1.0 : 1.0;

            if (dist <= DIST_HOLD) {
                hold = true;
            } else {
                double bearing = Math.atan2(dy, dx);
                double err = angleWrap(bearing - r);
                errF = angleWrap((1.0 - ERR_ALPHA) * errF + ERR_ALPHA * err);
                double absErrDeg = Math.abs(Math.toDegrees(errF));

                if (!hold && absErrDeg <= DEAD_ENTER_DEG) hold = true;
                if (hold && absErrDeg >= DEAD_EXIT_DEG) hold = false;

                if (!hold) {
                    double servoRaw = SERVO_CENTER + sign * (errF / Math.toRadians(SERVO_RANGE_DEG));
                    servoRaw = clamp(servoRaw, POS_RIGHT_LIMIT, POS_LEFT_LIMIT);
                    servoRaw = applyStickyLimits(servoRaw, currentServo);
                    targetServo = servoRaw;
                } else {
                    targetServo = currentServo;
                }
            }

            if (Math.abs(targetServo - currentServo) < 1e-6) {
                servoVel = approach(servoVel, 0.0, MAX_SERVO_ACCEL * dt);
            } else {
                double desiredVel = clamp((targetServo - currentServo) / dt, -MAX_SERVO_VEL, MAX_SERVO_VEL);
                servoVel = approach(servoVel, desiredVel, MAX_SERVO_ACCEL * dt);
            }

            double nextServo = currentServo + servoVel * dt;
            nextServo = clamp(nextServo, POS_RIGHT_LIMIT, POS_LEFT_LIMIT);
            nextServo = applyStickyLimits(nextServo, currentServo);

            tureta.setPosition(nextServo);

            telemetry.addData("x y r", "%.2f %.2f %.1f", x, y, Math.toDegrees(r));
            telemetry.addData("pos tureta", "%.3f", tureta.getPosition());
            telemetry.update();

        }
    }

    private static double angleWrap(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
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

    private double applyStickyLimits(double candidate, double current) {
        if (current <= POS_RIGHT_LIMIT + STICKY_MARGIN && candidate <= POS_RIGHT_LIMIT + STICKY_MARGIN) {
            return POS_RIGHT_LIMIT;
        }
        if (current >= POS_LEFT_LIMIT - STICKY_MARGIN && candidate >= POS_LEFT_LIMIT - STICKY_MARGIN) {
            return POS_LEFT_LIMIT;
        }
        return candidate;
    }
    public static double distanceToTarget(double x, double y) {
        return Math.hypot(x, y);
    }

}
