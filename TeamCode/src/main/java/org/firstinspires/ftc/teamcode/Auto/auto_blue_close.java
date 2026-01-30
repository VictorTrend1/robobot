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

        VelConstraint slow_vel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20),
                new AngularVelConstraint(Math.PI/2)
        ));
        AccelConstraint slow_acc = new ProfileAccelConstraint(-20, 30);



        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .afterTime(0, ()->{
                            ruleta.goTo(Ruleta.Slot.S1);
                            tureta.setPosition(0.26);
                        })
                        .strafeToLinearHeading(new Vector2d(-3, -54), Math.toRadians(93))
                        .build()
        );
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        timpRecunostinta.reset();
        timpRecunostinta.startTime();
        while(timpRecunostinta.seconds()<=1){
            LLResult result = limelight.getLatestResult();
            plan = getAprilTagId(result);

        }
        timpRecunostinta.reset();

        telemetry.addData("plan", plan);
        telemetry.update();
        if(plan==null){plan= Ruleta.Plan3.PPG;}

        shooter.spinUpTo(1400);
        tureta.setPosition(0.3);

        shootOnPlan(plan);


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble())))
                        .afterTime(0,()->{shooter.stopFlywheel();
                        intake.start();})

                        .strafeToLinearHeading(new Vector2d(-21, -41), Math.toRadians(136))

                        .afterTime(0, ()->{ new Thread (() -> {
                            intake.start();
                            ruleta.goTo(Ruleta.Slot.C1);
                            sleep(300);
                            while(!sensors.ballPresent()){}
                            ruleta.goTo(Ruleta.Slot.C2);
                            sleep(300);
                            while(!sensors.ballPresent()){}
                            ruleta.goTo(Ruleta.Slot.C3);
                            sleep(300);
                            while(!sensors.ballPresent()){}
                            ruleta.goTo(Ruleta.Slot.S1);
                        }).start();

                        })
                        .strafeToLinearHeading(new Vector2d(-25, -41), Math.toRadians(136),slow_vel,slow_acc)
                        .build());



        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .afterTime(0, ()->{
                            intake.stop();
                            shooter.spinUpTo(1400);
                        })
                        .strafeToLinearHeading(new Vector2d(-3, -54), Math.toRadians(45))
                        .build()
        );

        shootOnPlan(plan);


//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))
//                        .afterTime(0, ()->{shooter.stopFlywheel();})
//                        .strafeToLinearHeading(new Vector2d(15, 5), Math.toRadians(0))
//                        .build()
//        );



        sleep(30000);
        //shoot_3();
        /*
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(-40, 10))
                        .build()
        );

         */
    }
}
