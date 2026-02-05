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
    public Servo led;
    public DcMotorEx kicker;
    public static double DEG = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            kicker = hardwareMap.get(DcMotorEx.class,"kicker");
            kicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            kicker.setPower(putere);

            servo1 = hardwareMap.get(Servo.class,"ruleta1");
            servo2 = hardwareMap.get(Servo.class, "ruleta2");

            servo1.setPosition(DEG);
            servo2.setPosition(DEG);


            telemetry.addData("putere motor: ", kicker.getPower());
            telemetry.addData("velocity motor: ", kicker.getVelocity());
            telemetry.addData("Ruleta: ", servo1.getPosition());
            telemetry.update();
        }

    }
}


///  TODO inaltime
/// TODO 0.47 sus 0.02 jos

