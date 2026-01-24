package org.firstinspires.ftc.teamcode.temp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpMode.aim;
import org.firstinspires.ftc.teamcode.systems.Tureta;

@Config
@TeleOp(name= "setpozision", group= "Teleop")
public class setpozition extends LinearOpMode {
    public static double pozitie;
    public Servo servo1, servo2;
    public static double DEG = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        aim aim = new aim(new Tureta(hardwareMap));
        waitForStart();
        while(opModeIsActive()) {
            servo1 = hardwareMap.get(Servo.class, "tureta1");
            servo2 = hardwareMap.get(Servo.class, "tureta2");

            servo1.setPosition(            aim.turretDegreesToServo(DEG));
            servo2.setPosition(            aim.turretDegreesToServo(DEG));



            telemetry.addData("pozitie servo: ", servo1.getPosition());
            telemetry.update();
        }

    }
}


///  TODO inaltime
/// TODO 0.47 sus 0.02 jos

