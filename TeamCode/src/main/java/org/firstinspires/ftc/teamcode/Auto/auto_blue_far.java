package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.PinpointDrive;
@Autonomous(name = "auto_blue_far")
public class auto_blue_far extends BaseAuto {

    private PinpointDrive drive;

    @Override
    protected void onInit() {
        super.onInit();
        SHOOT_MIN_OK = 1700;
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
    }
    @Override
    protected void onRun() {

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .afterTime(0.1, ()->{shooter.spinUpTo(1600);})
                        .strafeToLinearHeading(new Vector2d(9, 2), Math.toRadians(25    ))
                        .build()
        );
        shoot_3();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble()))

                        .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(0))
                        .build()
        );
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
