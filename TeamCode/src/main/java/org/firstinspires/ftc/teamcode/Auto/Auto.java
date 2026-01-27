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
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        super.onInit();


        while(opModeInInit()){
            LLResult result = limelight.getLatestResult();
            plan = getAprilTagId(result);
        }

        SHOOT_MIN_OK = 1600;
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
    }
    @Override
    protected void onRun() {

        telemetry.addData("plan", plan);
        telemetry.update();


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .afterTime(0, ()->{shooter.spinUpTo(1650);
                            ruleta.goTo(Ruleta.Slot.S1);
                            tureta.setPosition(aim.turretDegreesToServo(-12));
                        })
                        .setTangent(Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(9, -2), Math.toRadians(90))
                        .build()
        );
        shootOnPlan(plan);
        shooter.stopFlywheel();
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