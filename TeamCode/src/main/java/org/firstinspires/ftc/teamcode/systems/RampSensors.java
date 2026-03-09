package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RampSensors {
    public Rev2mDistanceSensor senzorIntake1;
    public RevColorSensorV3 senzorIntake2;

    public RampSensors(HardwareMap hw) {
        senzorIntake1 = hw.get(Rev2mDistanceSensor.class, "senzorIntake");
        senzorIntake2 = hw.get(RevColorSensorV3.class, "senzorIntake2");

    }
    public boolean ballPresent( ) {
        return (senzorIntake1.getDistance(DistanceUnit.CM) <=7.0);
    }
    public boolean ballPresent2( ) {
        return (senzorIntake2.getDistance(DistanceUnit.CM) <=16.0);
    }

    public boolean isPurple( ) {
        return true;
    }
}
