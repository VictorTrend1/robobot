package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Tureta;

public class autoAim_TEST extends LinearOpMode {
    @Override
    public void runOpMode( ) throws InterruptedException {

        autoAim_Thread thread2Class = new autoAim_Thread(telemetry, hardwareMap);
        Thread thread2 = new Thread(thread2Class);
        autoAim_LL aimLL = new autoAim_LL(new Tureta(hardwareMap));

        PhotonCore.ExperimentalParameters ph = new PhotonCore.ExperimentalParameters();
        ph.setMaximumParallelCommands(8);
        ph.setSinglethreadedOptimized(false);
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(new Vector2d(0,0), Math.toRadians(0)));



        if (thread2.isAlive()) {
            thread2.interrupt();
            thread2.join();
        }

        waitForStart();
        thread2.start();

        while (!Thread.currentThread().isInterrupted()) {

            drive.setDrivePowers( new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ) );


        }
    }

}
