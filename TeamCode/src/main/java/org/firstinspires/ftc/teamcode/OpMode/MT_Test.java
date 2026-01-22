package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@TeleOp(name = "MT_Test")
public class MT_Test extends LinearOpMode {
    @Override
    public void runOpMode( ) throws InterruptedException {
        SecondaryThread thread2Class =
                new SecondaryThread(telemetry, hardwareMap);
        Thread thread2 =
                new Thread(thread2Class);

        PhotonCore photonCore = new PhotonCore();
        PhotonCore.ExperimentalParameters ph = new PhotonCore.ExperimentalParameters();
        ph.setMaximumParallelCommands(8);
        ph.setSinglethreadedOptimized(false);



        if (thread2.isAlive()) {
            thread2Class.stopThread();
            thread2.interrupt();
            thread2.join();
        }

        waitForStart();
        thread2.start();

        while (opModeIsActive()) {
            sleep(100000);


        }

        thread2Class.stopThread();
        thread2.interrupt();
        try {
            thread2.join(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


     class SecondaryThread implements Runnable {
        Telemetry telemetry;
        HardwareMap hm;
        volatile boolean isRunning = true;

        public SecondaryThread(Telemetry telemetry, HardwareMap hm) {
            this.telemetry = telemetry;
            this.hm = hm;
        }

         PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(new Vector2d(0,0), Math.toRadians(0)));


         @Override
        public void run( ) {
            while (!Thread.currentThread().isInterrupted()) {

                if(isRunning) {

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    ));

                }

                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
        public void stopThread( ) {
            isRunning = false;
        }

        public void continueThread( ) {
            isRunning = true;
        }
    }
}

