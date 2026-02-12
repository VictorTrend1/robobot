package org.firstinspires.ftc.teamcode.temp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(name= "setpozision", group= "Teleop")
public class setpozition extends LinearOpMode {
    public static double putere = 0;
    public Servo servo1, servo2;
    public Servo kicker;
    public static boolean sortator = false;
    public static boolean kick = false;
    public static boolean tureta = false;
    public static double DEG = 0;
    public Servo tureta1 , tureta2;



    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            if(kick){kicker = hardwareMap.get(Servo.class,"kicker");
            kicker.setPosition(DEG);
                telemetry.addData("kicker: ", kicker.getPosition());
            }
            if(sortator){
            servo1 = hardwareMap.get(Servo.class,"ruleta1");
            servo2 = hardwareMap.get(Servo.class, "ruleta2");

            servo1.setPosition(DEG);
            servo2.setPosition(DEG);
            telemetry.addData("Ruleta: ", servo1.getPosition());

            }
            if(tureta){
                tureta1 = hardwareMap.get(Servo.class,"tureta1");
                tureta2 = hardwareMap.get(Servo.class,"tureta2");
                tureta1.setPosition(DEG);
                tureta2.setPosition(DEG);
            }

            telemetry.update();
        }

    }
}


///  TODO inaltime
/// TODO 0.47 sus 0.02 jos

