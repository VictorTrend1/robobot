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


        while(opModeInInit()){
            LLResult result = limelight.getLatestResult();
            plan = getAprilTagId(result);
        }

        SHOOT_MIN_OK = 1600;
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));



    }
    @Override
    protected void onRun() {
        if(plan==null){
            plan = Ruleta.Plan3.PPG;
        }

        telemetry.addData("plan", plan);
        telemetry.update();

        VelConstraint slow_vel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20),
                new AngularVelConstraint(Math.PI/2)
        ));
        AccelConstraint slow_acc = new ProfileAccelConstraint(-5, 5);


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .afterTime(0, ()->{shooter.spinUpTo(1650);
                            ruleta.setPoz(Ruleta.SAFE);
                            ruleta.goTo(Ruleta.Slot.S1);
                            tureta.setPosition(0.545);
                        })
                        .strafeToLinearHeading(new Vector2d(9, 4), Math.toRadians(0))
                        .build()
        );
        sleep(300);
        intake.start();
        shootOnPlan(plan);
        shooter.stopFlywheel();
        intake.stop();
//        //=============INTAKE================
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))
                        .afterTime(0, ()->{intake.start();
                            ruleta.goTo(Ruleta.Slot.C1);
                            shooter.stopFlywheel();

                        })
                        .strafeToLinearHeading(new Vector2d(28, 22), Math.toRadians(89))
                        .build()
        );Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))
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
                            ruleta.setPoz(Ruleta.SLOT_S1);
                        }).start();

                        })
                        .strafeToLinearHeading(new Vector2d(28, 60), Math.toRadians(90),slow_vel,slow_acc)

                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))
                        .afterTime(0.1, ()->{
                            shooter.spinUpTo(1650);
                            intake.stop();})
                        .strafeToLinearHeading(new Vector2d(9, 4), Math.toRadians(0))
                        .build()
        );
        tureta.setPosition(0.55);
        intake.start();
        sleep(200);
        shootOnPlan(plan);
        intake.stop();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))
                        .strafeToLinearHeading(new Vector2d(9, 22), Math.toRadians(0))
                        .build()
        );
        PoseStorage.currentPose = drive.pose;
        TargetStorage.targetX = 138.34;
        TargetStorage.targetY = 66.759;
        TargetStorage.pipeline=3;
        shooter.stopFlywheel();

        sleep(30000);

    }
}