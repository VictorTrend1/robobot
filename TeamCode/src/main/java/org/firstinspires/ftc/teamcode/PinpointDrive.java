package org.firstinspires.ftc.teamcode;
//0.85 y offset
//15.5 y offset in sus
//9.2
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.zyxOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

/**
 * Experimental extension of MecanumDrive that uses the Gobilda Pinpoint sensor for localization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the MIT License by Gobilda (Base 10 Assets, LLC)
 * Unless otherwise noted, comments are from Gobilda
 */
@Config
public class PinpointDrive extends MecanumDrive {
    @Config
    public static class Params {

        public String pinpointDeviceName = "pinpoint";

        public double xOffset = -3.890;  // -6.8
        public double yOffset = -3.950; // -3.4
        public double encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public boolean usePinpointIMUForTuning = true;
    }

    public static Params PARAMS = new Params();
    public GoBildaPinpointDriverRR pinpoint;
    private Pose2d lastPinpointPose = pose;

    public PinpointDrive(HardwareMap hardwareMap, Pose2d pose , boolean resetPinpoint) {
        super(hardwareMap, pose);
        FlightRecorder.write("PINPOINT_PARAMS",PARAMS);
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class,PARAMS.pinpointDeviceName);

        if (PARAMS.usePinpointIMUForTuning) {
            lazyImu = new LazyImu(hardwareMap, PARAMS.pinpointDeviceName, new RevHubOrientationOnRobot(zyxOrientation(0, 0, -0.3346457)));
        }

        pinpoint.setOffsets(DistanceUnit.MM.fromInches(PARAMS.xOffset), DistanceUnit.MM.fromInches(PARAMS.yOffset));


        pinpoint.setEncoderResolution(PARAMS.encoderResolution);

        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);
        if(!resetPinpoint){
            pinpoint.update();
            pose = pinpoint.getPositionRR();

        }
        else{
            pinpoint.resetPosAndIMU();
        }
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        pinpoint.setPosition(pose);
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastPinpointPose != pose) {
            pinpoint.setPosition(pose);
        }
        pinpoint.update();
        pose = pinpoint.getPositionRR();
        lastPinpointPose = pose;

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
        FlightRecorder.write("PINPOINT_RAW_POSE",new FTCPoseMessage(pinpoint.getPosition()));
        FlightRecorder.write("PINPOINT_STATUS",pinpoint.getDeviceStatus());

        return pinpoint.getVelocityRR();
    }


    // for debug logging
    public static final class FTCPoseMessage {
        public long timestamp;
        public double x;
        public double y;
        public double heading;

        public FTCPoseMessage(Pose2D pose) {
            this.timestamp = System.nanoTime();
            this.x = pose.getX(DistanceUnit.INCH);
            this.y = pose.getY(DistanceUnit.INCH);
            this.heading = pose.getHeading(AngleUnit.RADIANS);
        }
    }



}