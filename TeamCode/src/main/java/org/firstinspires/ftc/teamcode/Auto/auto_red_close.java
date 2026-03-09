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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.PoseStorage;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.TargetStorage;

import java.util.Arrays;

@Autonomous(name = "auto_red_close")
public class auto_red_close extends BaseAuto {

    private PinpointDrive drive;
    public   ElapsedTime timpRecunostinta = new ElapsedTime();


    @Override
    protected void onInit() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        super.onInit();
        MecanumDrive.PARAMS.TimpPlus = 0.0;

        TargetStorage.pipeline = 2;
        TargetStorage.targetX = -0.7;
        TargetStorage.targetY = -12.3;

        SHOOT_MIN_OK = 1415;
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0), true);    }
    @Override
    protected void onRun() {
        VelConstraint slow_vel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint slow_acc = new ProfileAccelConstraint(-5, 10);
        AccelConstraint slow_acc2 = new ProfileAccelConstraint(-5, 10);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.toDouble())))
                        .afterTime(0.1, () -> {
                            shooter.spinUpTo(1435);
                            ruleta.goTo(Ruleta.Slot.S1);
                            tureta.setPosition(0.85);
                            intake.start();
                        })
                        //.strafeToLinearHeading(new Vector2d(-54, -4.24), Math.toRadians(0))
                                               .strafeToLinearHeading(new Vector2d(-53, -12.3), Math.toRadians(0))
                        .build()
        );
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        timpRecunostinta.reset();
        timpRecunostinta.startTime();
        while(timpRecunostinta.seconds()<=2){
            LLResult result = limelight.getLatestResult();
            plan = getAprilTagId(result);
        }
        if(plan == null){
            plan = Ruleta.Plan3.PPG;
            telemetry.addLine("plan: null");
        }
        telemetry.addData("plan: ", plan);
        telemetry.update();
        timpRecunostinta.reset();
        tureta.setPosition(0.52);
        sleep(100);
        shootOnPlan(plan);
        ruleta.goTo(Ruleta.Slot.C1);
        shooter.stopFlywheel();
        sleep(100);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), drive.pose.heading.toDouble()))
                        .strafeToLinearHeading(new Vector2d(-45.6, -12.3), Math.toRadians(-45))
                        .afterTime(0, () -> {
                            new Thread(() -> {
                                intake.start();
                                sleep(100);
                                while(!sensors.ballPresent()){}
                                ruleta.goTo(Ruleta.Slot.C2);
                                sleep(100);
                                while(!sensors.ballPresent()){}
                                ruleta.goTo(Ruleta.Slot.C3);
                                sleep(100);
                            }).start();
                        })
                        .strafeToLinearHeading(new Vector2d(-30.76, -30.33), Math.toRadians(-45), slow_vel, slow_acc)
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), drive.pose.heading.toDouble()))
                        .afterTime(0, () -> {
                            shooter.spinUpTo(1435);
                            tureta.setPosition(0.75);
                        })
                       // .strafeToLinearHeading(new Vector2d(-59, -4.24), Math.toRadians(-45))
                        .strafeToLinearHeading(new Vector2d(-51, -10.3), Math.toRadians(-45))

                        .build()
        );
        shootOnPlan(plan);
        shooter.stopFlywheel();
        ruleta.goTo(Ruleta.Slot.C1);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble())))
                        .strafeToLinearHeading(new Vector2d(-55.6, -40.5), Math.toRadians(-45))
                        .afterTime(0, () -> {
                            new Thread(() -> {
                                intake.start();
                                sleep(100);
                                while(!sensors.ballPresent()){}
                                ruleta.goTo(Ruleta.Slot.C3);
                                sleep(100);
                                while(!sensors.ballPresent()){}
                                ruleta.goTo(Ruleta.Slot.C2);
                                sleep(100);
                            }).start();
                        })
                        .strafeToLinearHeading(new Vector2d(-46.3, -50.9), Math.toRadians(-45), slow_vel, slow_acc2)
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), drive.pose.heading.toDouble()))
                        .afterTime(0.1, () -> {
                            tureta.setPosition(0.75);
                            shooter.spinUpTo(1435);
                        })
                        //.strafeToLinearHeading(new Vector2d(-56, -4.24), Math.toRadians(-45))
                        .strafeToLinearHeading(new Vector2d(-60.62, -14.0), Math.toRadians(-45))

                        .build()
        );

        shootOnPlan(plan);
        intake.stop();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .afterTime(0, () -> { shooter.stopFlywheel(); })
                        .strafeToLinearHeading(new Vector2d(-49.2, -48.2), Math.toRadians(-45))
                        .build()
        );

        PoseStorage.currentPose = drive.pose;

        sleep(30000);
    }
}