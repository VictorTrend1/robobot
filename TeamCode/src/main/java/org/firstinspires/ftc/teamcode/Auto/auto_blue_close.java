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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Ruleta;

import java.util.Arrays;

@Autonomous(name = "auto_blue_close")
public class auto_blue_close extends BaseAuto {

    private PinpointDrive drive;
    public ElapsedTime timpRecunostinta = new ElapsedTime();
    private static final double TARGET_X = 0.0;
    private static final double TARGET_Y = 0.0;

    @Override
    protected void onInit() {
        super.onInit();
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    public double getDistance() {
        double robotX = drive.pose.position.x;
        double robotY = drive.pose.position.y;
        double deltaX = TARGET_X - robotX;
        double deltaY = TARGET_Y - robotY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    private void spinUpDynamic() {
        double dist = getDistance();
        shooter.setRPMForDistance(dist);
        SHOOT_MIN_OK = (int)(shooter.getVelocity() * 0.97);
        telemetry.update();
    }

    @Override
    protected void onRun() {
        plan = Ruleta.Plan3.PPG;

        VelConstraint slow_vel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint slow_acc = new ProfileAccelConstraint(-30, 60);
        AccelConstraint slow_acc2 = new ProfileAccelConstraint(-50, 90);

        // === SHOOT 1 ===
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0.1, () -> {
                            spinUpDynamic();
                            ruleta.goTo(Ruleta.Slot.S1);
                        })
                        .strafeToLinearHeading(new Vector2d(-33.27, 16.69), Math.toRadians(1.5))
                        .build()
        );

        tureta.setPosition(0.5);
        intake.start();
        spinUpDynamic();
        shootOnPlan(plan);
        intake.stop();
        shooter.stopFlywheel();
        ruleta.goTo(Ruleta.Slot.C1);

        // === INTAKE 1 ===
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble())))
                        .strafeToLinearHeading(new Vector2d(-36.7, 26), Math.toRadians(45))
                        .afterTime(0, () -> {
                            new Thread(() -> {
                                intake.start();
                                ruleta.goTo(Ruleta.Slot.C1);
                                sleep(200);
                                while (!sensors.ballPresent()) {}
                                ruleta.goTo(Ruleta.Slot.C2);
                                sleep(200);
                                while (!sensors.ballPresent()) {}
                                ruleta.goTo(Ruleta.Slot.C3);
                                sleep(200);
                                while (!sensors.ballPresent()) {}
                                ruleta.goTo(Ruleta.Slot.S1);
                            }).start();
                        })
                        .strafeToLinearHeading(new Vector2d(-11.74, 50.37), Math.toRadians(45), slow_vel, slow_acc)
                        .build()
        );

        // === SHOOT 2 ===
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toRadians(drive.pose.heading.toDouble())))
                        .afterTime(0, () -> {
                            intake.stop();
                            spinUpDynamic();
                        })
                        .strafeToLinearHeading(new Vector2d(-36.7, 23.3), Math.toRadians(45))
                        .build()
        );

        tureta.setPosition(0.345);
        intake.start();
        spinUpDynamic();
        shootOnPlan(plan);
        intake.stop();
        shooter.stopFlywheel();
        ruleta.goTo(Ruleta.Slot.C1);

        // === INTAKE 2 ===
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble())))
                        .strafeToLinearHeading(new Vector2d(-55.4, 42.5), Math.toRadians(45))
                        .afterTime(0, () -> {
                            new Thread(() -> {
                                intake.start();
                                ruleta.goTo(Ruleta.Slot.C1);
                                sleep(200);
                                while (!sensors.ballPresent()) {}
                                ruleta.goTo(Ruleta.Slot.C2);
                                sleep(200);
                                while (!sensors.ballPresent()) {}
                                ruleta.goTo(Ruleta.Slot.C3);
                                sleep(200);
                                while (!sensors.ballPresent()) {}
                                ruleta.goTo(Ruleta.Slot.S1);
                            }).start();
                        })
                        .strafeToLinearHeading(new Vector2d(-19.22, 84.8), Math.toRadians(45), slow_vel, slow_acc2)
                        .build()
        );

        // === SHOOT 3 ===
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble())))
                        .afterTime(0.1, () -> {
                            intake.stop();
                            spinUpDynamic();
                        })
                        .strafeToLinearHeading(new Vector2d(-45.7, 26.3), Math.toRadians(45))
                        .build()
        );

        tureta.setPosition(0.345);
        intake.start();
        spinUpDynamic();
        shootOnPlan(plan);
        intake.stop();

        sleep(30000);
    }
}