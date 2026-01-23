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
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.RampSensors;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Tureta;

import java.util.Arrays;


@Autonomous(name = "auto_test")
public class auto_test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        VelConstraint vel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(120),
                new AngularVelConstraint(Math.PI/2)
        ));
        AccelConstraint acc = new ProfileAccelConstraint(-50, 80);
        Pose2d start = new Pose2d(0,0, Math.toRadians(0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, start);

        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(start)

                        .strafeToLinearHeading(new Vector2d(48,12), Math.toRadians(90), vel, acc)
                        .build());
        sleep(4000);
    }

}
