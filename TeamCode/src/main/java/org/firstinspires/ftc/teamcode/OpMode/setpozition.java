package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name= "setpozision", group= "Teleop")
public class setpozition extends LinearOpMode {
    public static double pozitie;
    public Servo servo1;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            servo1 = hardwareMap.get(Servo.class, "inaltime");
            servo1.setPosition(pozitie);
            telemetry.addData("pozitie servo: ", servo1.getPosition());
            telemetry.update();
        }

    }
}


///  TODO inaltime
/// TODO 0.47 sus 0.02 jos

