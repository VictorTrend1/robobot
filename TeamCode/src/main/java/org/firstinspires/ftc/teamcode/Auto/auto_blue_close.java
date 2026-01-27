package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.recunoastere.Limelight.getAprilTagId;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Ruleta;

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



        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .afterTime(0, ()->{shooter.spinUpTo(1400);
                            ruleta.goTo(Ruleta.Slot.S1);
//                            tureta.setPosition(aim.turretDegreesToServo(27));
                            tureta.setPosition(0.5);
                        })
                        .strafeToLinearHeading(new Vector2d(58, -5), Math.toRadians(126))
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
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble())))
                        .strafeToLinearHeading(new Vector2d(47, -6), Math.toRadians(-176))
                        .build());
        shootOnPlan(plan);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d(drive.pose.position.x, drive.pose.position.y), Math.toDegrees(drive.pose.heading.toDouble())))
                        .strafeToLinearHeading(new Vector2d(36, -6), Math.toRadians(-178))
                        .build());



        sleep(300);

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
