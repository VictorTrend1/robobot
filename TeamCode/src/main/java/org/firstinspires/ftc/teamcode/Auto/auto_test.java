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

@Autonomous(name = "auto_test")
public class auto_test extends BaseAuto {

    private PinpointDrive drive;

    @Override
    protected void onInit() {
         drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0) , true);
    }



    @Override
    protected void onRun() {


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                        .strafeToLinearHeading(new Vector2d(50,0), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d( 50,20 ), Math.toRadians(-90))
                        .build()
        );
        sleep(30000);
    }
}