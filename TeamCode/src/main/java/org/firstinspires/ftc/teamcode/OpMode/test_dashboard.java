package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Tureta;

@TeleOp(name="teleop_dashnoard" , group = "teleop")
@Config
public class test_dashboard extends LinearOpMode {
    Shooter shooter = new Shooter(hardwareMap);
    Tureta tureta = new Tureta(hardwareMap);
    Ruleta ruleta = new Ruleta(hardwareMap);
    Ruleta.Slot currsentScore = Ruleta.Slot.S1;

@Override
    public void runOpMode() throws InterruptedException{

    while  (opModeIsActive()){
        tureta.goDefault();
        teleop.Edge dpadLeftEdge = new teleop.Edge();
        if (dpadLeftEdge.rising(gamepad1.right_bumper)) {
            currsentScore = nextScoreSlot(currsentScore);
            ruleta.goTo(currsentScore);
        }
        if(gamepad1.square){
            shooter.spinUp();
            ruleta.goTo(Ruleta.Slot.S1);
        }
        if(gamepad1.cross){
            shooter.kicker.setPower(1);
        }



    }
}
    private Ruleta.Slot nextCollectSlot(Ruleta.Slot s) {
        if (s == Ruleta.Slot.C1) return Ruleta.Slot.C2;
        if (s == Ruleta.Slot.C2) return Ruleta.Slot.C3;
        return Ruleta.Slot.C1;
    }
    public Ruleta.Slot nextScoreSlot(Ruleta.Slot s) {
        if (s == Ruleta.Slot.S1) return Ruleta.Slot.S2;
        if (s == Ruleta.Slot.S2) return Ruleta.Slot.S3;
        return Ruleta.Slot.S1;
    }
}
