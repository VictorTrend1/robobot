package org.firstinspires.ftc.teamcode.temp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.PinpointDrive;
@Config
@TeleOp(name="Pinpoint Offset Tuner")
public class PinpointOffsetTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        double xOffsetInch = -3.69;
        double yOffsetInch = -3.94;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0),true);

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            if (gamepad1.dpad_up) {xOffsetInch += 0.01; sleep(100);}
            if (gamepad1.dpad_down) {xOffsetInch -= 0.01; sleep(100);}
            if (gamepad1.dpad_right) {yOffsetInch += 0.01; sleep(100);}
            if (gamepad1.dpad_left) {yOffsetInch -= 0.01; sleep(100);}


            // Reset poziție
            if (gamepad1.start) {
                drive.pinpoint.resetPosAndIMU();
                try { Thread.sleep(300); } catch (InterruptedException e) {}
            }

            drive.pinpoint.setOffsets(xOffsetInch * 25.4, yOffsetInch * 25.4);  // convertește la mm


            drive.updatePoseEstimate();

            telemetry.addLine("=== PINPOINT OFFSET TUNER ===");
            telemetry.addData("X Offset", "%.3f inch (%.1f mm)", xOffsetInch, xOffsetInch * 25.4);
            telemetry.addData("Y Offset", "%.3f inch (%.1f mm)", yOffsetInch, yOffsetInch * 25.4);
            telemetry.addLine();
            telemetry.addData("Poziție X", "%.2f inch", drive.pose.position.x);
            telemetry.addData("Poziție Y", "%.2f inch", drive.pose.position.y);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addLine("ROTEȘTE ROBOTUL PE LOC 360°");
            telemetry.addLine("X și Y trebuie să rămână ~0");
            telemetry.addLine();
            telemetry.addLine("D-pad: ±0.01 | XYAB: ±0.001");
            telemetry.addLine("START = reset poziție");
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}