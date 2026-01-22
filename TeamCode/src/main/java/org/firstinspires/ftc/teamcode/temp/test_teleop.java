package org.firstinspires.ftc.teamcode.temp;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name= "test_teleop" , group =  "Teleop")
public class test_teleop extends LinearOpMode {
    public DcMotorEx shooter1, shooter2;
    @Override
    public void runOpMode() throws InterruptedException{
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){

            shooter1.setPower(gamepad1.right_trigger);
            shooter2.setPower(shooter1.getPower());


            telemetry.addData("shooter1: ", shooter1.getVelocity());
            telemetry.addData("shooter2: ", shooter2.getVelocity());
            telemetry.addData("shooter1 pow: ", shooter1.getPower());
            telemetry.addData("shooter2 pow", shooter2.getPower());
            telemetry.update();
        }
    }
}
