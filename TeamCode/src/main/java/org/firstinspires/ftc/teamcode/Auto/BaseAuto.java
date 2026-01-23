package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Tureta;

public abstract class BaseAuto extends LinearOpMode {

    protected Shooter shooter;
    protected Ruleta ruleta;
    protected Tureta tureta;
    protected Intake intake;

    protected int SHOOT_MIN_OK = 1500;
    protected int WAIT_MS = 200;

    @Override
    public final void runOpMode() {
        shooter = new Shooter(hardwareMap);
        ruleta  = new Ruleta(hardwareMap);
        tureta  = new Tureta(hardwareMap);
        intake  = new Intake(hardwareMap);


        onInit();

        waitForStart();
        if (isStopRequested()) return;

        onRun();
    }

    protected void onInit() {
        ruleta.goTo(Ruleta.Slot.S1);
        shooter.retractKicker();
    }
    protected abstract void onRun();
    protected final void waitShooterAtSpeed(double minOk) {
        while (opModeIsActive() && !shooter.atSpeedTo(SHOOT_MIN_OK)) {

        }
    }

    protected final void shoot_3() {
        waitShooterAtSpeed(SHOOT_MIN_OK);
        shooter.pushKicker();
        sleep(WAIT_MS);
        shooter.retractKicker();
        sleep(100);
        shooter.pushKicker();
        sleep(WAIT_MS);
        shooter.retractKicker();
        sleep(WAIT_MS);

        ruleta.goTo(Ruleta.Slot.S2);
        waitShooterAtSpeed(SHOOT_MIN_OK);
        sleep(WAIT_MS);
        shooter.pushKicker();
        sleep(WAIT_MS);
        shooter.retractKicker();
        sleep(100);
        shooter.pushKicker();
        sleep(WAIT_MS);
        shooter.retractKicker();
        sleep(WAIT_MS);

        ruleta.goTo(Ruleta.Slot.S3);
        waitShooterAtSpeed(SHOOT_MIN_OK);
        sleep(WAIT_MS);
        shooter.pushKicker();
        sleep(WAIT_MS);
        shooter.retractKicker();
        sleep(100);
        shooter.pushKicker();
        sleep(WAIT_MS);
        shooter.retractKicker();
        sleep(100);

    }
}
