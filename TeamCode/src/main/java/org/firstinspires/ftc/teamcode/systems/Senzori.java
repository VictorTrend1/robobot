package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Senzori {

    public enum culori_bile {
        VERDE, MOV, UNK
    }

    public RevColorSensorV3 senzorIntake;
    public RevColorSensorV3 senzorOuttake;

    private static final int INTAKE_LIGHT_THRESHOLD = 180;

    private static final int GREEN_DOMINANCE_MIN = 20;
    private static final int PURPLE_RB_OVER_G_MIN = 20;

    public static final int COLOR_SAMPLE_MS = 80;

    public Senzori(HardwareMap hardwareMap) {
        senzorIntake = hardwareMap.get(RevColorSensorV3.class, "senzorIntake");
        senzorOuttake = hardwareMap.get(RevColorSensorV3.class, "senzorOuttake");
    }

    public boolean mingeLaIntake() {
        int lumina = senzorIntake.alpha();
        return lumina < INTAKE_LIGHT_THRESHOLD;
    }

    public culori_bile citesteCuloareOuttakeOData() {
        int r = senzorOuttake.red();
        int g = senzorOuttake.green();
        int b = senzorOuttake.blue();

        if (g > r + GREEN_DOMINANCE_MIN && g > b + GREEN_DOMINANCE_MIN) {
            return culori_bile.VERDE;
        }

        if (r > g + PURPLE_RB_OVER_G_MIN && b > g + PURPLE_RB_OVER_G_MIN) {
            return culori_bile.MOV;
        }

        return culori_bile.UNK;
    }

    public culori_bile citesteCuloareOuttakeStabil() {
        long start = System.currentTimeMillis();
        int verde = 0, mov = 0, nec = 0;

        while (System.currentTimeMillis() - start < COLOR_SAMPLE_MS) {
            culori_bile c = citesteCuloareOuttakeOData();
            if (c == culori_bile.VERDE) verde++;
            else if (c == culori_bile.MOV) mov++;
            else nec++;
        }

        if (verde > mov && verde > nec) return culori_bile.VERDE;
        if (mov > verde && mov > nec) return culori_bile.MOV;
        return culori_bile.UNK;
    }
}