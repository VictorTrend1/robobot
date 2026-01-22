package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RampSensors {
    public RevColorSensorV3 senzorIntake1;
    public RevColorSensorV3 senzorIntake2;

    public RampSensors(HardwareMap hw) {
        senzorIntake1 = hw.get(RevColorSensorV3.class, "senzorIntake");
        senzorIntake2 = hw.get(RevColorSensorV3.class, "senzorIntake2");

    }
    public boolean ballPresent( ) {
        return (senzorIntake1.getDistance(DistanceUnit.CM) <=2.9);
    }

    public boolean isPurple( ) {
        return true;
    }
}
