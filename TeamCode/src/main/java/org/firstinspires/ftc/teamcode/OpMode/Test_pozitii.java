package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.PoseStorage;

@TeleOp(name = "Pose test")
public class Test_pozitii extends LinearOpMode {

    @Override
    public void runOpMode() {

        PinpointDrive drive = new PinpointDrive(hardwareMap, PoseStorage.currentPose);

        waitForStart();

        while (opModeIsActive()) {


            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            Pose2d pose = drive.pose;

            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading",pose.heading);
            telemetry.update();
        }
    }
}
