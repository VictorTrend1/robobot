package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.systems.Senzori;

@Config
@TeleOp(name="test_senzori", group = "Teleop")
public class test_senzori  extends LinearOpMode {
    public RevColorSensorV3 senzorIntake1;
    public RevColorSensorV3 senzorIntake2;

    @Override
    public void runOpMode() throws InterruptedException{
        senzorIntake1 = hardwareMap.get(RevColorSensorV3.class, "senzorIntake");
        senzorIntake2 = hardwareMap.get(RevColorSensorV3.class, "senzorIntake2");
        waitForStart();
        while(opModeIsActive()){
                telemetry.addData("senzor1: ", senzorIntake1.getDistance(DistanceUnit.CM));
                telemetry.addData("senzor2: ", senzorIntake2.getDistance(DistanceUnit.CM));
                telemetry.update();
        }
    }
}
