package org.firstinspires.ftc.teamcode.recunoastere;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.List;

@TeleOp
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Pipeline-ul pentru AprilTag
        limelight.pipelineSwitch(1);

        waitForStart();

        limelight.start();
        while(opModeInInit()) {
        }

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            String tagId = getAprilTagId(result);

            telemetry.addData("AprilTag ID", tagId);
            telemetry.update();




        }
    }

    public static String getAprilTagId(LLResult result) {

        if (result == null || !result.isValid()) return "null";

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return "null";

        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();

            if (id == 21) {
                return "GPP";
            }else if(id==22){
                return "PGP";
            } else if (id==23) {
                return "PPG";
            }
        }

        return "null";
    }
}
