package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.RampSensors;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class Teleop extends LinearOpMode {

    private enum State { INTAKE, SCORE }

    private static final Ruleta.Plan3 SCORE_PLAN = Ruleta.Plan3.GPG;

    private static final double TOGGLE_THRESHOLD = 0.6;
    private static final double SHOOT_THRESHOLD  = 0.6;

    // Timpi mecanici (servo kicker)
    private static final long KICK_PUSH_MS = 140;
    private static final long KICK_RETRACT_MS = 140;

    @Override
    public void runOpMode() {

        Ruleta ruleta = new Ruleta(hardwareMap);

        Intake intake = new Intake(hardwareMap);
        intake.setRuleta(ruleta);

        Shooter shooter = new Shooter(hardwareMap);
        RampSensors sensors = new RampSensors(hardwareMap);

        State state = State.INTAKE;

        boolean intakeForwardOn = false;
        boolean intakeReverseOn = false;

        Edge rtEdge = new Edge();
        Edge ltEdge = new Edge();
        Edge ballEdge = new Edge();
        Edge shootEdge = new Edge();

        Ruleta.Slot currentScoreSlot = null;
        int shotsDone = 0;

        // INIT
        shooter.stopAll();
        intake.resetForIntake();
        shooter.safeForRuletaRotate();
        ruleta.goTo(Ruleta.Slot.C1);

        waitForStart();

        while (opModeIsActive()) {

            switch (state) {

                case INTAKE: {

                    shooter.safeForRuletaRotate();

                    boolean rtPressed = gamepad1.right_trigger > TOGGLE_THRESHOLD;
                    boolean ltPressed = gamepad1.left_trigger > TOGGLE_THRESHOLD;

                    if (rtEdge.rising(rtPressed)) {
                        intakeForwardOn = !intakeForwardOn;
                        if (intakeForwardOn) intakeReverseOn = false;
                    }

                    if (ltEdge.rising(ltPressed)) {
                        intakeReverseOn = !intakeReverseOn;
                        if (intakeReverseOn) intakeForwardOn = false;
                    }

                    if (intakeForwardOn) intake.start();
                    else if (intakeReverseOn) intake.reverse();
                    else intake.stop();

                    boolean ballNow = sensors.ballPresent();
                    if (ballEdge.rising(ballNow)) {
                        boolean isGreen = !sensors.isPurple();
                        intake.onBallEntered(isGreen);

                        Ruleta.Slot next = ruleta.firstFreeCollectSlot();
                        if (next != null) {
                            shooter.safeForRuletaRotate();
                            ruleta.goTo(next);
                            sleep(120);
                        }
                    }

                    if (intake.isReadyForScore()) {
                        intake.stop();
                        intakeForwardOn = false;
                        intakeReverseOn = false;

                        ruleta.setPlan(SCORE_PLAN);

                        ruleta.moveToScore(Ruleta.Slot.C1, Ruleta.Slot.S1);
                        ruleta.moveToScore(Ruleta.Slot.C2, Ruleta.Slot.S2);
                        ruleta.moveToScore(Ruleta.Slot.C3, Ruleta.Slot.S3);

                        shotsDone = 0;
                        currentScoreSlot = null;

                        shooter.retractKicker();
                        shooter.spinUp();
                        sleep(250);

                        shootEdge.reset(false);
                        state = State.SCORE;
                    }

                    telemetry.addData("STATE", "INTAKE");
                    telemetry.addData("Balls", intake.getBallsIntaked());
                    telemetry.addData("Ruleta", ruleta.debug());
                    telemetry.update();
                    break;
                }


                case SCORE: {

                    intake.stop();
                    shooter.spinUp();

                    if (currentScoreSlot == null) {
                        currentScoreSlot = pickNextScoreSlot(ruleta);
                        if (currentScoreSlot == null) {
                            resetToIntake(shooter, intake, ruleta, rtEdge, ltEdge, ballEdge, shootEdge);
                            state = State.INTAKE;
                            break;
                        }
                    }

                    shooter.safeForRuletaRotate();
                    ruleta.goTo(currentScoreSlot);
                    sleep(120);

                    boolean shootPressed = gamepad1.right_trigger > SHOOT_THRESHOLD;
                    if (shootEdge.rising(shootPressed) && shooter.atSpeed()) {

                        shooter.pushKicker();
                        sleep(KICK_PUSH_MS);

                        shooter.retractKicker();
                        sleep(KICK_RETRACT_MS);

                        ruleta.popScoredBall(currentScoreSlot);
                        shotsDone++;

                        currentScoreSlot = pickNextScoreSlot(ruleta);

                        if (shotsDone >= 3 || currentScoreSlot == null) {
                            resetToIntake(shooter, intake, ruleta, rtEdge, ltEdge, ballEdge, shootEdge);
                            state = State.INTAKE;
                        }
                    }

                    telemetry.addData("STATE", "SCORE");
                    telemetry.addData("Vel", shooter.getVelocity());
                    telemetry.addData("AtSpeed", shooter.atSpeed());
                    telemetry.addData("TargetSlot", currentScoreSlot);
                    telemetry.addData("ShotsDone", shotsDone);
                    telemetry.addData("Ruleta", ruleta.debug());
                    telemetry.update();
                    break;
                }
            }
        }

        shooter.stopAll();
        intake.stop();
    }

    private static Ruleta.Slot pickNextScoreSlot(Ruleta r) {
        Ruleta.Slot s = r.nextScoreSlotFromPlan();
        if (s != null) return s;
        if (!r.isEmpty(Ruleta.Slot.S1)) return Ruleta.Slot.S1;
        if (!r.isEmpty(Ruleta.Slot.S2)) return Ruleta.Slot.S2;
        if (!r.isEmpty(Ruleta.Slot.S3)) return Ruleta.Slot.S3;
        return null;
    }

    private static void resetToIntake(
            Shooter shooter,
            Intake intake,
            Ruleta ruleta,
            Edge rt, Edge lt, Edge ball, Edge shoot
    ) {
        shooter.stopAll();
        intake.resetForIntake();
        shooter.safeForRuletaRotate();
        ruleta.goTo(Ruleta.Slot.C1);

        rt.reset(false);
        lt.reset(false);
        ball.reset(false);
        shoot.reset(false);
    }

    private static final class Edge {
        private boolean prev = false;

        boolean rising(boolean now) {
            boolean r = now && !prev;
            prev = now;
            return r;
        }

        void reset(boolean value) {
            prev = value;
        }
    }
}
