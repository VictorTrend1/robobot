package org.firstinspires.ftc.teamcode.recunoastere;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Ruleta;

import java.util.List;

@TeleOp
public class distanta_goal extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(new Vector2d(0,0), Math.toRadians(0)),true);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);

        limelight.start();
        while(opModeInInit()) {
            limelight.start();
        }
        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();


            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);

                double targetOffsetAngle_Vertical = ty;
                // how many degrees back is your limelight rotated from perfectly vertical?
                double limelightMountAngleDegrees = 0;
                // distance from the center of the Limelight lens to the floor
                double limelightLensHeightInches = 7.87;
                // distance from the target to the floor
                double goalHeightInches = 26.0;
                double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
                //calculate distance
                double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
                telemetry.addData("Distance", getDistance(result.getTa()));
                telemetry.addData("Target area", result.getTa());
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
            telemetry.update();
        }

    }
    public double getDistance(double ta){
        double scale = 42.12636166 ;
        double distance = (scale/ta);
        return distance;

    }

}
