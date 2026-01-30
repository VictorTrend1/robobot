package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.Tureta;

public class autoAim_Thread implements Runnable {
    Telemetry telemetry;
    HardwareMap hm;
    volatile boolean isRunning = true;

    public autoAim_Thread(Telemetry telemetry, HardwareMap hm) {
        this.telemetry = telemetry;
        this.hm = hm;
    }




    @Override
    public void run( ) {

        autoAim_LL aim = new autoAim_LL(new Tureta( hm ) );

        while (!Thread.currentThread().isInterrupted()) {

            aim.updateAim();


            try {
                Thread.sleep( 20 );
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }
}
