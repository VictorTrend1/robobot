package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.Math.TargetTracker;
import org.firstinspires.ftc.teamcode.systems.Tureta;

@TeleOp(name = "TrackTest")
public class TrackTest extends LinearOpMode {

    private static final int LIMELIGHT_PIPELINE = 2;

    private static final double START_X = 0.0;
    private static final double START_Y = 0.0;
    private static final double START_HEADING_DEG = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        PinpointDrive drive = new PinpointDrive(
                hardwareMap,
                new Pose2d(new Vector2d(START_X, START_Y), Math.toRadians(START_HEADING_DEG)),true
        );
        Tureta tureta = new Tureta(hardwareMap);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        TargetTracker.Params ap = new TargetTracker.Params();

        ap.servoCenter = 0.5;

        ap.servoLeft = 0.36;
        ap.servoRight = 0.64;

        ap.servoMinLimit = 0.34;
        ap.servoMaxLimit = 0.66;


        ap.corrMax = 1.50;

        ap.kP = -2.0;
        ap.kD = -2.5;

        ap.trackMaxServoSpeed = 2.5;
        ap.snapMaxServoSpeed = 3.0;

        TargetTracker aimer = new TargetTracker(ap);
        tureta.setPosition(ap.servoCenter);

        boolean lastA = false;
        boolean lastB = false;
        boolean lastX = false;

        long lastNs = System.nanoTime();

        waitForStart();
        if (isStopRequested()) return;

        limelight.start();
        aimer.setTarget(124.87,-59.00);

        while (opModeIsActive()) {



            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            long nowNs = System.nanoTime();
            double dt = (nowNs - lastNs) * 1e-9;
            lastNs = nowNs;
            if (dt < 0.008) dt = 0.008;
            if (dt > 0.050) dt = 0.050;

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            if (a && !lastA) aimer.requestAim();
            if (b && !lastB) aimer.cancel();
            if (x && !lastX) {
                aimer.reset();
                tureta.setPosition(ap.servoCenter);
            }

            lastA = a;
            lastB = b;
            lastX = x;

            Pose2d pose = drive.pose;

            LLResult r = limelight.getLatestResult();
            boolean hasVision = (r != null) && r.isValid();
            double tx = hasVision ? r.getTx() : 0.0;

            double servoCmd = aimer.update(
                    drive.pose.position.x,
                    drive.pose.position.y,
                    pose.heading.toDouble(),
                    hasVision,
                    tx,
                    dt
            );

            tureta.setPosition(servoCmd);
            drive.updatePoseEstimate();
            telemetry.addData("HasVision Count", hasVision ? "YES" : "NO");

            telemetry.addData("Mode", aimer.getMode());
            telemetry.addData("Locked", aimer.isLocked());
            telemetry.addData("Pose X", "%.2f", drive.pose.position.x);
            telemetry.addData("Pose Y", "%.2f", drive.pose.position.y);
            telemetry.addData("HeadingDeg", "%.2f", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Vision", hasVision);
            telemetry.addData("tx", "%.2f", tx);
            telemetry.addData("ServoCmd", "%.4f", servoCmd);
            telemetry.addLine("A=aim  B=cancel  X=reset");
            telemetry.update();

            sleep(20);
        }
    }
}
