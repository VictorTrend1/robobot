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
    public   ElapsedTime timpRecunostinta = new ElapsedTime();



    @Override
    protected void onInit() {

        super.onInit();


        SHOOT_MIN_OK = 1350;
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
    }
    @Override
    protected void onRun() {
        plan= Ruleta.Plan3.PPG;
        VelConstraint slow_vel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20),
                new AngularVelConstraint(Math.PI/2)
        ));
        AccelConstraint slow_acc = new ProfileAccelConstraint(-30, 60);
        AccelConstraint slow_acc2 = new ProfileAccelConstraint(-50, 90);




        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))
                        .afterTime(0.1, ()->{shooter.spinUpTo(1400);
                            ruleta.goTo(Ruleta.Slot.S1);
                        })
                        .strafeToLinearHeading(new Vector2d(-48.27,1.69),Math.toRadians(1.5) )
                        .build()
        );

        tureta.setPosition(0.5);
        intake.start();
        shootOnPlan(plan);
        intake.stop();
        shooter.stopFlywheel();
        //=============INTAKE================

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble())))
                        .strafeToLinearHeading(new Vector2d( -51.7, 11), Math.toRadians(45))
                        .afterTime(0, ()->{ new Thread (() -> {
                            intake.start();
                            ruleta.goTo(Ruleta.Slot.C1);
                            sleep(200);
                            while(!sensors.ballPresent()){}
                            ruleta.goTo(Ruleta.Slot.C2);
                            sleep(200);
                            while(!sensors.ballPresent()){}
                            ruleta.goTo(Ruleta.Slot.C3);
                            sleep(200);
                            while(!sensors.ballPresent()){}
                            ruleta.goTo(Ruleta.Slot.S1);
                        }).start();

                        })
                        .strafeToLinearHeading(new Vector2d(-26.74, 35.37),Math.toRadians(45) ,slow_vel , slow_acc)
                        .build());


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toRadians(drive.pose.heading.toDouble())))
                        .afterTime(0, ()->{
                            intake.stop();
                            shooter.spinUpTo(1400);
                        })
                        .strafeToLinearHeading(new Vector2d(-51.7,8.3),Math.toRadians(45))
                        .build()
        );
        tureta.setPosition(0.345);
        intake.start();
        shootOnPlan(plan);
        intake.stop();
// SPIKE 2
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toRadians(drive.pose.heading.toDouble())))
//                        .afterTime(0, ()->{intake.start();
//                            ruleta.goTo(Ruleta.Slot.C1);
//                            shooter.stopFlywheel();
//
//                        })
//                       // .strafeToConstantHeading(new Vector2d(-70.4,-26.5)).strafeToLinearHeading(new Vector2d(-35.35, -28.65), 0)
//                        .strafeToLinearHeading(new Vector2d(-70.4,-26.5),Math.toRadians(-45))
//                        .build()
//        );
        shooter.stopFlywheel();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble())))
                        .strafeToLinearHeading(new Vector2d(-70.4,27.5),Math.toRadians(45))

                        .afterTime(0, ()->{ new Thread (() -> {
                            intake.start();
                            ruleta.goTo(Ruleta.Slot.C1);
                            sleep(200);
                            while(!sensors.ballPresent()){}
                            ruleta.goTo(Ruleta.Slot.C2);
                            sleep(200);
                            while(!sensors.ballPresent()){}
                            ruleta.goTo(Ruleta.Slot.C3);
                            sleep(200);
                            while(!sensors.ballPresent()){}
                            ruleta.goTo(Ruleta.Slot.S1);
                        }).start();

                        })
                        .strafeToLinearHeading(new Vector2d(-34.22, 69.8),Math.toRadians(45) , slow_vel , slow_acc2)
                        .build());
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble())))
                        .afterTime(0.1, ()->{
                            intake.stop();
                            shooter.spinUpTo(1400);
                        })
                        .strafeToLinearHeading(new Vector2d(-60.7,11.3),Math.toRadians(45))
                        .build()
        );
        tureta.setPosition(0.345);
        intake.start();
        shootOnPlan(plan);
        intake.stop();
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))
//                        .afterTime(0, ()->{shooter.stopFlywheel();})
//                        .strafeToConstantHeading(new Vector2d(-40, -28.65))
//                        .build()
//        );



        sleep(30000);

    }
}
