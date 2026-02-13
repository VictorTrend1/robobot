package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.recunoastere.Limelight.getAprilTagId;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Math.TargetTracker;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.PoseStorage;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.TargetStorage;

import java.util.Arrays;

@Autonomous(name = "auto_red_far")
public class auto_red_far extends BaseAuto {

    private PinpointDrive drive;
    private AutoAim autoAim;


    @Override
    protected void onInit() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        super.onInit();


        while (opModeInInit()) {
            LLResult result = limelight.getLatestResult();
            plan = getAprilTagId(result);
        }

        SHOOT_MIN_OK = 1600;
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

        autoAim = new RedFarAutoAim();
        autoAim.init(hardwareMap, drive);


    }

    @Override
    protected void onRun() {
        if (plan == null) {
            plan = Ruleta.Plan3.PPG;
        }

        telemetry.addData("plan", plan);
        telemetry.update();

        VelConstraint slow_vel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint slow_acc = new ProfileAccelConstraint(-20, 30);


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .afterTime(0, () -> {
                            shooter.spinUpTo(1650);
                            ruleta.setPoz(Ruleta.SLOT_S1);
                            autoAim.startAiming();
                            intake.start();
                        })
                        .strafeToLinearHeading(new Vector2d(9, -4), Math.toRadians(0))
                        .build()
        );
        autoAim.aimAndWait(100);
        sleep(200);
        intake.start();
        shootOnPlan(plan);
        intake.stop();
        shooter.stopFlywheel();
        autoAim.cancel();
        //=============INTAKE================
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0, () -> {
                            intake.start();
                            ruleta.goTo(Ruleta.Slot.C1);
                            shooter.stopFlywheel();

                        })
                        .strafeToLinearHeading(new Vector2d(28, -22), Math.toRadians(-89))
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0, () -> {
                            new Thread(() -> {
                                intake.start();
                                ruleta.goTo(Ruleta.Slot.C1);
                                sleep(200);
                                while (!sensors.ballPresent()) {
                                }
                                ruleta.goTo(Ruleta.Slot.C2);
                                sleep(200);
                                while (!sensors.ballPresent()) {
                                }
                                ruleta.goTo(Ruleta.Slot.C3);
                                sleep(200);
                                while (!sensors.ballPresent()) {
                                }
                                ruleta.setPoz(Ruleta.SLOT_S1);
                            }).start();

                        })
                        .strafeToLinearHeading(new Vector2d(28, -50), Math.toRadians(-90), slow_vel, slow_acc)

                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0.1, () -> {
                            autoAim.startAiming();
                            shooter.spinUpTo(1650);
                            intake.stop();
                        })
                        .strafeToLinearHeading(new Vector2d(9, -4), Math.toRadians(0))
                        .build()
        );
        autoAim.aimAndWait(100);
        intake.start();
        sleep(100);
        shootOnPlan(plan);
        intake.stop();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0.1, () -> {
                            intake.stop();
                            shooter.stopFlywheel();
                        })
                        .strafeToLinearHeading(new Vector2d(9, -20), Math.toRadians(0))
                        .build()
        );
        PoseStorage.currentPose = drive.pose;
        sleep(30000);

    }

    private class RedFarAutoAim extends AutoAim {
        @Override
        protected TargetTracker.Params getAimerParams() {
            TargetTracker.Params ap = new TargetTracker.Params();
            ap.servoCenter = 0.5;

            ap.servoLeft = 0.37;
            ap.servoRight = 0.63;

            ap.servoMinLimit = 0.37;
            ap.servoMaxLimit = 0.63;

            pipeline = 2;

            targetX = 137.23;
            targetY = -66.539;

            ap.corrMax = 0.60;
            ap.txAlpha = 0.05;

                        TargetStorage.targetX = targetX;
            TargetStorage.targetY = targetY;
            TargetStorage.pipeline = pipeline;
            return ap;
        }
    }
}

