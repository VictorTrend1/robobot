package org.firstinspires.ftc.teamcode.temp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Auto.auto_blue_close;
import org.firstinspires.ftc.teamcode.Math.TuretaAutoAim;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Tureta;

@Config
@TeleOp(name="Shooter RPM Tuning")
public class ShooterTuning extends LinearOpMode {

    public static int TARGET_RPM = 0;
    protected Limelight3A limelight;

    @Override
    public void runOpMode() {

        Shooter shooter = new Shooter(hardwareMap);

        Tureta tureta = new Tureta(hardwareMap);

        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(new Vector2d(-144, -60), Math.toRadians(0)));

        TuretaAutoAim autoAim = new TuretaAutoAim(hardwareMap, drive);

        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(2);

        limelight.start();

        tureta.goDefault();
        autoAim.setTarget(0.0, 0.0);


        waitForStart();

        while (opModeIsActive()) {
            tureta.goDefault();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            if(gamepad1.cross){
                shooter.pushKicker();
                sleep(350);
                shooter.retractKicker();
            }
            if(gamepad1.square){
                shooter.spinUpTo(TARGET_RPM);
            }


            telemetry.addData("Current RPM", shooter.getVelocity());
            telemetry.addData("Distance", autoAim.getDistance());
            telemetry.addData("robot heading error: ", autoAim.getHeadingError());
            telemetry.addData("Heading Error", autoAim.getHeadingError());
            telemetry.addData("Is Aimed", autoAim.isAimed(3.0));
            telemetry.update();

            previousGamepad1.copy(gamepad1);

            sleep(20);
        }

        shooter.stopAll();
    }
}