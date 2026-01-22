package org.firstinspires.ftc.teamcode.temp_auto;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Inaltime;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.RampSensors;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Tureta;

import java.util.Arrays;


@Autonomous(name = "auto_blue_far")
public class auto_blue_far extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        VelConstraint vel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(120),
                new AngularVelConstraint(Math.PI/2)
        ));
        AccelConstraint acc = new ProfileAccelConstraint(-50, 80);

        Pose2d start = new Pose2d(0,0, Math.toRadians(0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, start);


        Ruleta ruleta = new Ruleta(hardwareMap);
        Tureta tureta =  new Tureta(hardwareMap);

        Intake intake = new Intake(hardwareMap);
        intake.setRuleta(ruleta);

        Shooter shooter = new Shooter(hardwareMap);
        RampSensors sensors = new RampSensors(hardwareMap);
        Inaltime inaltime = new Inaltime(hardwareMap);


        shooter.stopAll();
        intake.resetForIntake();
        shooter.safeForRuletaRotate();
        ruleta.goTo(Ruleta.Slot.S1);


        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(start)
                        .afterTime(0, ()->{
                            tureta.goDefault();
                            shooter.spinUpTo(1700);
                        })
                        .strafeToLinearHeading(new Vector2d(10,0), Math.toRadians(52), vel, acc)
                        .afterTime(0.1, ()->{ new Thread(()->{

                            while (!shooter.atSpeedTo(1650)){}
                            ruleta.goTo(Ruleta.Slot.S1);

                            kick_ball(shooter);
                            sleep(450);

                            ruleta.goTo(Ruleta.Slot.S2);
                            while (!shooter.atSpeedTo(1650)){}

                            sleep(300);
                            kick_ball(shooter);
                            sleep(300);

                            ruleta.goTo(Ruleta.Slot.S3);
                            while (!shooter.atSpeedTo(1650)){}

                            sleep(300);
                            kick_ball(shooter);

                        }).start();})
                        .build());
        sleep(4000);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble())))
                        .afterTime(0, ()->{ new Thread(()->{
                            tureta.goDefault();
                            shooter.stopFlywheel();
                            ruleta.goTo(Ruleta.Slot.C1);
                        }).start();})
                        .strafeToConstantHeading(new Vector2d(24.7,35) )




                        .build());
        sleep(10000);
    }

    private void kick_ball(Shooter shooter) {
        shooter.pushKicker();
        sleep(200);

        shooter.retractKicker();
        sleep(200);

        shooter.pushKicker();
        sleep(200);

        shooter.retractKicker();

    }
}
