package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.recunoastere.Limelight.getAprilTagId;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Ruleta;

@Autonomous(name = "auto")
public class Auto extends BaseAuto {

    private PinpointDrive drive;


    @Override
    protected void onInit() {

        SHOOT_MIN_OK = 1600;
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0) , true);
    }
    @Override
    protected void onRun() {



        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))

                        .setTangent(Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(40, -24), Math.toRadians(90))
                        .build()
        );

        //=============INTAKE================
/*
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))
                        .afterTime(0, ()->{intake.start();
                            shooter.stopFlywheel();
                            ruleta.goTo(Ruleta.Slot.C1);
                        })
                        .strafeToLinearHeading(new Vector2d(26, -25), Math.toRadians(-90))

                        .build()
        );
        ruleta.goTo(Ruleta.Slot.C2);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))
                        .strafeToLinearHeading(new Vector2d(26, -39), Math.toRadians(-89))
                        .build()
        );
        ruleta.goTo(Ruleta.Slot.C3);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))
                        .strafeToLinearHeading(new Vector2d(30, -55), Math.toRadians(-89))
                        .build()
        );
        ruleta.goTo(Ruleta.Slot.S1);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))
                        .afterTime(0.1, ()->{shooter.spinUpTo(1650);
                            intake.stop();})
                        .strafeToLinearHeading(new Vector2d(9, -2), Math.toRadians(0))
                        .build()
        );
        shootOnPlan(plan);

 */

        sleep(30000);

    }
}