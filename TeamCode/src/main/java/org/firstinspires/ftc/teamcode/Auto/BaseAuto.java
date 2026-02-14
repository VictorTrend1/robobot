package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.RampSensors;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Tureta;

public abstract class BaseAuto extends LinearOpMode {

    protected Shooter shooter;
    protected Ruleta ruleta;
    protected Tureta tureta;
    protected Intake intake;
    protected int SHOOT_MIN_OK;

    protected RampSensors sensors;

    Ruleta.Plan3 plan;

    protected int balls;
    protected int WAIT_MS = 200;

    @Override
    public final void runOpMode() {
        shooter = new Shooter(hardwareMap);
        ruleta  = new Ruleta(hardwareMap);
        tureta  = new Tureta(hardwareMap);
        intake  = new Intake(hardwareMap);
        sensors = new RampSensors(hardwareMap);
        balls=0;

        onInit();

        waitForStart();
        if (isStopRequested()) return;

        onRun();
    }

    protected void onInit() {
        ruleta.goTo(Ruleta.Slot.S1);
        //shooter.retractKicker();
        tureta.goDefault();
        SHOOT_MIN_OK = 1650;
    }
    protected abstract void onRun();
    protected final void waitShooterAtSpeed(double minOk) {
        while (true) {if (!opModeIsActive() || shooter.atSpeedTo( SHOOT_MIN_OK )) break;}
    }

    protected final void shoot_3() {
        waitShooterAtSpeed(SHOOT_MIN_OK);
        sleep(WAIT_MS);
        sleep(WAIT_MS);


        ruleta.goTo(Ruleta.Slot.S2);
        waitShooterAtSpeed(SHOOT_MIN_OK);
        sleep(WAIT_MS);
        sleep(WAIT_MS);
        sleep(WAIT_MS);


        ruleta.goTo(Ruleta.Slot.S3);
        waitShooterAtSpeed(SHOOT_MIN_OK);
        sleep(WAIT_MS);
        //shooter.pushKicker();
        sleep(WAIT_MS);
        //shooter.retractKicker();


    }

    protected final void preparePreloadGreenInS1PurpleInS2S3(Ruleta.Plan3 plan) {
        ruleta.clear();

        ruleta.onBallIntake(false);
        ruleta.onBallIntake(false);
        ruleta.onBallIntake(true);

        ruleta.setPlan(plan);

        ruleta.moveToScore(Ruleta.Slot.C1, Ruleta.Slot.S1);
        ruleta.moveToScore(Ruleta.Slot.C2, Ruleta.Slot.S2);
        ruleta.moveToScore(Ruleta.Slot.C3, Ruleta.Slot.S3);
    }
    protected final void shootOnPlan(Ruleta.Plan3 plan) {

        preparePreloadGreenInS1PurpleInS2S3(plan);
        sleep(250);

        int shotsDone = 0;

        while (opModeIsActive() && shotsDone < 3) {

            Ruleta.Slot slot = pickNextScoreSlot(ruleta);
            if (slot == null) break;

            ruleta.goTo(slot);
            sleep(300);

            while (opModeIsActive() && !shooter.atSpeedTo(SHOOT_MIN_OK)) {

            }
            sleep(200);
            kick();
            sleep(200);
            ruleta.popScoredBall(slot);
            shotsDone++;
        }
    }

    protected final void kick() {
        shooter.pushKicker();
        sleep(250);
        shooter.retractKicker();
    }

    protected final Ruleta.Slot pickNextScoreSlot(Ruleta r) {
        Ruleta.Slot s = r.nextScoreSlotFromPlan();
        if (s != null) return s;
        if (!r.isEmpty(Ruleta.Slot.S1)) return Ruleta.Slot.S1;
        if (!r.isEmpty(Ruleta.Slot.S2)) return Ruleta.Slot.S2;
        if (!r.isEmpty(Ruleta.Slot.S3)) return Ruleta.Slot.S3;
        return null;
    }
}
