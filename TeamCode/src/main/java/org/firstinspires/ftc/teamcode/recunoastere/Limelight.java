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
import org.firstinspires.ftc.teamcode.systems.Ruleta;

import java.util.List;

@TeleOp
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        waitForStart();

        limelight.start();
        while(opModeInInit()) {
        }
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            Ruleta.Plan3 tagId = getAprilTagId(result);
            telemetry.addData("AprilTag ID", tagId);
            telemetry.update();
        }
    }

    public static Ruleta.Plan3 getAprilTagId(LLResult result) {
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();

            if (id == 21) {
                return Ruleta.Plan3.GPP;
            }else if(id==22){
                return Ruleta.Plan3.PGP;
            } else if (id==23) {
                return Ruleta.Plan3.PPG;
            }
        }

        return null;
    }
}
