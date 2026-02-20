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
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.PoseStorage;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.TargetStorage;

import java.util.Arrays;

@Autonomous(name = "auto_blue_far")
public class auto_blue_far extends BaseAuto {

    private PinpointDrive drive;


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

        drive = new PinpointDrive(hardwareMap, new Pose2d(-144, -60, 0));
    }

    public double getDistance() {
        double robotX = drive.pose.position.x;
        double robotY = drive.pose.position.y;
        double deltaX = 0 - robotX;
        double deltaY = 0 - robotY;
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
        if (plan == null) {
            plan = Ruleta.Plan3.PPG;
        }

        telemetry.addData("plan", plan);
        telemetry.update();

        VelConstraint slow_vel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(170),
                new AngularVelConstraint(3* Math.PI)
        ));
        AccelConstraint slow_acc = new ProfileAccelConstraint(-5, 10);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-144, -60, 0))
                        .afterTime(0, () -> {
                            spinUpDynamic();
                            ruleta.setPoz(Ruleta.SLOT_S1);
                            tureta.setPosition(0.63);
                            intake.start();
                        })
                        .strafeToLinearHeading(new Vector2d(-135, -56), Math.toRadians(0))
                        .build()
        );
        sleep(300);
        intake.start();

        spinUpDynamic();
        tureta.setPosition(0.63);
        sleep(100);
        shootOnPlan(plan);

        intake.stop();
        shooter.stopFlywheel();

//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
//                        .afterTime(0, () -> {
//                            intake.start();
//                            ruleta.goTo(Ruleta.Slot.C1);
//                            shooter.stopFlywheel();
//                        })
//                        .strafeToLinearHeading(new Vector2d(-116, -38), Math.toRadians(90))
//                        .build()
//        );

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .strafeToLinearHeading(new Vector2d(-116, -38), Math.toRadians(90))
                        .afterTime(0, () -> {
                            new Thread(() -> {
                                intake.start();
                                ruleta.goTo(Ruleta.Slot.C1);
                                sleep(150);
                                while (!sensors.ballPresent()) {}
                                ruleta.goTo(Ruleta.Slot.C2);
                                sleep(150);
                                while (!sensors.ballPresent()) {}
                                ruleta.goTo(Ruleta.Slot.C3);
                                sleep(150);
                                while (!sensors.ballPresent()) {}
                                ruleta.setPoz(Ruleta.SLOT_S1);
                            }).start();
                        })
                        .strafeToLinearHeading(new Vector2d(-116, -25), Math.toRadians(90), slow_vel, slow_acc)
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0.1, () -> {
                            spinUpDynamic();
                            intake.stop();
                        })
                        .splineToLinearHeading(new Pose2d(-135, -56, Math.toRadians(0)), Math.toRadians(90))
                        .build()
        );

        tureta.setPosition(0.63);
        intake.start();

        spinUpDynamic();
        sleep(100);
        shootOnPlan(plan);

        intake.stop();

        /// SPIKE 2
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
//                        .afterTime(0, () -> {
//                            intake.start();
//                            ruleta.goTo(Ruleta.Slot.C1);
//                            shooter.stopFlywheel();
//                        })
//                        .strafeToLinearHeading(new Vector2d(-92, -38), Math.toRadians(90))
//                        .build()
//        );

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .strafeToLinearHeading(new Vector2d(-92, -38), Math.toRadians(90))
                        .afterTime(0, () -> {
                            new Thread(() -> {
                                intake.start();
                                ruleta.goTo(Ruleta.Slot.C1);
                                sleep(150);
                                while (!sensors.ballPresent()) {}
                                ruleta.goTo(Ruleta.Slot.C2);
                                sleep(150);
                                while (!sensors.ballPresent()) {}
                                ruleta.goTo(Ruleta.Slot.C3);
                                sleep(150);
                                while (!sensors.ballPresent()) {}
                                ruleta.setPoz(Ruleta.SLOT_S1);
                            }).start();
                        })
                        .strafeToLinearHeading(new Vector2d(-92, -25), Math.toRadians(90), slow_vel, slow_acc)
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0.1, () -> {
                            spinUpDynamic();
                            intake.stop();
                        })
                       // .strafeToLinearHeading(new Vector2d(-135, -56), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-135, -56, Math.toRadians(0)), Math.toRadians(90))
                        .build()
        );
        tureta.setPosition(0.63);
        intake.start();

        spinUpDynamic();
        sleep(100);
        shootOnPlan(plan);



        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0.1, () -> {
                            intake.stop();
                            shooter.stopFlywheel();
                            ruleta.goTo(Ruleta.Slot.C1);
                        })
                        .strafeToLinearHeading(new Vector2d(-135, -40), Math.toRadians(0))
                        .build()
        );

        PoseStorage.currentPose = drive.pose;
        TargetStorage.targetX = 0.0;
        TargetStorage.targetY = 0.0;
        TargetStorage.pipeline = 2;
        shooter.stopFlywheel();

        sleep(30000);
    }
}