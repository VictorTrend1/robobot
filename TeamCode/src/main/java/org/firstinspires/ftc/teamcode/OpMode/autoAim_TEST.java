package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.Tureta;

@TeleOp(name = "aimtest")
public class autoAim_TEST extends LinearOpMode {
    @Override
    public void runOpMode( ) throws InterruptedException {



        PhotonCore.ExperimentalParameters ph = new PhotonCore.ExperimentalParameters();
        ph.setMaximumParallelCommands(8);
        ph.setSinglethreadedOptimized(false);



//        if (thread2.isAlive()) {
//            thread2.interrupt();
//            thread2.join();
//        }


        autoAim_LL aim = new autoAim_LL(new Tureta( hardwareMap ) );
        aim.init(hardwareMap, 2);
        aim.start();
        waitForStart();
        //thread2.start();

        while (!Thread.currentThread().isInterrupted()) {

            aim.updateAim();
            telemetry.addData("TY:  ", aim.limelight.getLatestResult().getTy());
            telemetry.addData("TX: ", aim.limelight.getLatestResult().getTx());
            telemetry.update();


        }
    }

}
