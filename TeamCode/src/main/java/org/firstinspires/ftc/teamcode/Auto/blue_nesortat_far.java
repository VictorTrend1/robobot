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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.PoseStorage;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.TargetStorage;

import java.lang.annotation.Target;
import java.util.Arrays;

@Autonomous(name = "blue_nesortat_far")
public class blue_nesortat_far extends BaseAuto {

    private PinpointDrive drive;


    @Override
    protected void onInit() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        super.onInit();
        MecanumDrive.PARAMS.TimpPlus = 0;
        SHOOT_MIN_OK = 1640;


        while (opModeInInit()) {
            LLResult result = limelight.getLatestResult();
            plan = getAprilTagId(result);
        }
        TargetStorage.pipeline = 3;
        TargetStorage.targetX= 0.0;
        TargetStorage.targetY = 0.0;

        drive = new PinpointDrive(hardwareMap, new Pose2d(-130, -61, 0) , true);
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
            plan = Ruleta.Plan3.PPP;
        }

        telemetry.addData("plan", plan);
        telemetry.update();

        VelConstraint slow_vel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30),
                new AngularVelConstraint(Math.PI)
        ));
        VelConstraint slow_vel2 = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel),
                new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel)
        ));

        AccelConstraint slow_acc = new ProfileAccelConstraint(-10, 20);
        AccelConstraint slow_acc2 = new ProfileAccelConstraint(-65, 100);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-130, -61, 0))
                        .afterTime(0, () -> {
                            spinUpDynamic();
                            ruleta.setPoz(Ruleta.SLOT_S1);
                            intake.start();
                            shooter.spinUpTo(1650);
                        })
                        .strafeToLinearHeading(new Vector2d(-120, -53), Math.toRadians(0))
                        .build()
        );
        sleep(300);
        intake.start();
        tureta.setPosition(0.6);
        sleep(100);
        shootOnPlan(plan);

        intake.stop();
        shooter.stopFlywheel();
        ruleta.goTo(Ruleta.Slot.C1);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .strafeToLinearHeading(new Vector2d(-123, -10.2), Math.toRadians(115))
                        .afterTime(0, () -> {
                            new Thread(() -> {
                                intake.start();
                                sleep(40);
                                while(!sensors.ballPresent()){}
                                ruleta.goTo(Ruleta.Slot.C2);
                                sleep(40);

                            }).start();
                        })
                        .strafeToLinearHeading(new Vector2d(-125, -8), Math.toRadians(101),slow_vel,slow_acc)
                        .strafeToLinearHeading(new Vector2d(-131, -20.8), Math.toRadians(90))
                        .afterTime(0, () -> {
                            ruleta.goTo(Ruleta.Slot.C3);
                        })
                        .strafeToLinearHeading(new Vector2d(-131, -8), Math.toRadians(90),slow_vel,slow_acc)
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .splineToLinearHeading(new Pose2d(-120, -51, Math.toRadians(90)), Math.toRadians(45))
                        .afterTime(0, () -> {
                            intake.reverse();
                            shooter.spinUpTo(1670);
                            tureta.setPosition(0.14);

                        })
                        .build()
        );

        intake.start();
        sleep(300);
        shootOnPlan(plan);

        intake.stop();
        shooter.stopAll();

        intake.start();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0, () -> {
                            ruleta.goTo(Ruleta.Slot.C1);
                            intake.start();
                        })
                        .strafeToLinearHeading(new Vector2d(-129, -40), Math.toRadians(120))
                        .afterTime(0, () -> {
                            new Thread(() -> {
                                intake.start();
                                ruleta.goTo(Ruleta.Slot.C1);
                                while (!sensors.ballPresent()) {}
                                sleep(100);
                                ruleta.goTo(Ruleta.Slot.C2);
                                while (!sensors.ballPresent()) {}
                                sleep(100);
                                ruleta.goTo(Ruleta.Slot.C3);

                            }).start();
                        })
                        .strafeToLinearHeading(new Vector2d(-129, -7), Math.toRadians(120),slow_vel2, slow_acc2)
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0, () -> {
                            ruleta.goTo(Ruleta.Slot.S1);
                            intake.reverse();
                            shooter.spinUp();
                        })
                        .splineToLinearHeading(new Pose2d(-120, -51, Math.toRadians(90)), Math.toRadians(45))
                        .build()
        );
        intake.start();
        sleep(300);
        shootOnPlan(plan);

        intake.stop();
        shooter.stopAll();

        intake.start();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0, () -> {
                            ruleta.goTo(Ruleta.Slot.C1);
                            intake.start();
                        })
                        .strafeToLinearHeading(new Vector2d(-126, -40), Math.toRadians(120))
                        .afterTime(0, () -> {
                            new Thread(() -> {
                                intake.start();
                                ruleta.goTo(Ruleta.Slot.C1);
                                while (!sensors.ballPresent()) {}
                                sleep(100);
                                ruleta.goTo(Ruleta.Slot.C2);
                                while (!sensors.ballPresent()) {}
                                sleep(100);
                                ruleta.goTo(Ruleta.Slot.C3);;


                            }).start();
                        })
                        .strafeToLinearHeading(new Vector2d(-126, -7), Math.toRadians(120), slow_vel2 , slow_acc2)
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0, () -> {
                            ruleta.goTo(Ruleta.Slot.S1);
                            intake.reverse();
                            shooter.spinUp();
                        })
                        .splineToLinearHeading(new Pose2d(-120, -51, Math.toRadians(90)), Math.toRadians(45))
                        .build()
        );
        tureta.setPosition(0.14);
        intake.start();

        shootOnPlan(plan);

        intake.stop();
        shooter.stopAll();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0.1, () -> {
                            intake.stop();
                            shooter.stopFlywheel();
                            ruleta.goTo(Ruleta.Slot.C1);
                        })
                        .strafeToLinearHeading(new Vector2d(-120, -41), Math.toRadians(0))
                        .build()
        );


        sleep(30000);
    }
}