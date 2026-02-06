package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Tureta;

@Config
@TeleOp(name="teleop_dashboard")
public class test_dashboard extends LinearOpMode {
    private Shooter shooter;
    private Tureta tureta;
    private Ruleta ruleta;
    private Intake intake;

    private Ruleta.Slot currentCollectSlot = Ruleta.Slot.C1;
    private Ruleta.Slot currentScoreSlot = Ruleta.Slot.S1;

    private teleop.Edge leftBumper = new teleop.Edge();
    private teleop.Edge rightBumper = new teleop.Edge();

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        tureta = new Tureta(hardwareMap);
        ruleta = new Ruleta(hardwareMap);
        intake = new Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            tureta.goDefault();
            if (leftBumper.rising(gamepad1.left_bumper)) {
                currentCollectSlot = nextCollectSlot(currentCollectSlot);
                ruleta.goTo(currentCollectSlot);
            }
            if (rightBumper.rising(gamepad1.right_bumper)) {
                currentScoreSlot = nextScoreSlot(currentScoreSlot);
                ruleta.goTo(currentScoreSlot);
            }

            if (gamepad1.square) {
                shooter.spinUp();
            } else {
                shooter.stopFlywheel();
            }

            if (gamepad1.left_trigger != 0) {
                intake.start();
            } else {
                intake.stop();
            }
        }
    }

    private Ruleta.Slot nextCollectSlot(Ruleta.Slot s) {
        if (s == Ruleta.Slot.C1) return Ruleta.Slot.C2;
        if (s == Ruleta.Slot.C2) return Ruleta.Slot.C3;
        return Ruleta.Slot.C1;
    }

    private Ruleta.Slot nextScoreSlot(Ruleta.Slot s) {
        if (s == Ruleta.Slot.S1) return Ruleta.Slot.S2;
        if (s == Ruleta.Slot.S2) return Ruleta.Slot.S3;
        return Ruleta.Slot.S1;
    }
}