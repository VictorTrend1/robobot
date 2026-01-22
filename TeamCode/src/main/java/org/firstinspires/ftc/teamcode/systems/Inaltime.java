package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Inaltime {
    private Servo inaltime;
    private double pozitie = 0.47;
    public Inaltime(HardwareMap hw){
        inaltime = hw.get(Servo.class,"inaltime");
        inaltime.setPosition(pozitie);
    }

}
