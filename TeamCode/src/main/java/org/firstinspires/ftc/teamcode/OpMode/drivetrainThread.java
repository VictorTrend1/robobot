package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class drivetrainThread implements Runnable {
    Telemetry telemetry;
    HardwareMap hm;
    volatile boolean isRunning = true;

    public drivetrainThread(Telemetry telemetry, HardwareMap hm) {
        this.telemetry = telemetry;
        this.hm = hm;
    }

    PinpointDrive drive = new PinpointDrive(hm, new Pose2d(new Vector2d(0,0), Math.toRadians(0)));


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
}
