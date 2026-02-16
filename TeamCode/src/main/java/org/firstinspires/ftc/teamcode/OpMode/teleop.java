package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.Math.TuretaAutoAim;
import org.firstinspires.ftc.teamcode.systems.PoseStorage;
import org.firstinspires.ftc.teamcode.systems.RampSensors;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.TargetStorage;
import org.firstinspires.ftc.teamcode.systems.Tureta;

@TeleOp(name = "teleop")
public class teleop extends LinearOpMode {

    private enum State { INTAKE, SCORE }
    private Servo led;

    private static final Ruleta.Plan3 SCORE_PLAN = Ruleta.Plan3.PPP;

    private volatile State currentState = State.INTAKE;
    private volatile boolean shouldAim = false;

    HardwareMap hm;

    @Override
    public void runOpMode() throws InterruptedException {

        Shooter shooter = new Shooter(hardwareMap);
        drivetrainThread thread2Class = new drivetrainThread(telemetry, hardwareMap,shooter);
        Thread thread2 = new Thread(thread2Class);


        PhotonCore.ExperimentalParameters ph = new PhotonCore.ExperimentalParameters();
        ph.setMaximumParallelCommands(8);
        ph.setSinglethreadedOptimized(false);

        if (thread2.isAlive()) {
            thread2.interrupt();
            thread2.join();
        }


        boolean lastDpadDown = false;

        led = hardwareMap.get(Servo.class, "led");

        Ruleta ruleta = new Ruleta(hardwareMap);

        Intake intake = new Intake(hardwareMap);
        intake.setRuleta(ruleta);

        RampSensors sensors = new RampSensors(hardwareMap);
        Edge circleEdge = new Edge();

        Edge rtEdge = new Edge();
        Edge ltEdge = new Edge();
        Edge ballEdge = new Edge();
        Edge shootEdge = new Edge();
        Edge triangleEdge = new Edge();

        Ruleta.Slot currentScoreSlot = null;
        Ruleta.Slot currentCollectSlot = Ruleta.Slot.C1;
        int shotsDone = 0;
        boolean aimToggle = false;

        // INIT
        shooter.stopAll();
        intake.resetForIntake();
        sleep(300);
        ruleta.goTo(Ruleta.Slot.C1);
        while(opModeInInit()){
            telemetry.addData("pipeline: ", TargetStorage.pipeline);
            telemetry.addData("Target x: ", TargetStorage.targetX);
            telemetry.addData("Target y: ", TargetStorage.targetY);
            telemetry.addData("x: ", PoseStorage.currentPose.position.x);
            telemetry.addData("y: ", PoseStorage.currentPose.position.y);
            telemetry.addData("Heading: ", PoseStorage.currentPose.heading);
            telemetry.update();
        }

        waitForStart();

        thread2.start();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger != 0) intake.start();
            else if (gamepad1.left_trigger != 0) intake.reverse();
            else intake.stop();

            Edge left_bumper = new Edge();
            if (left_bumper.rising(gamepad1.left_bumper)) {
                currentCollectSlot = nextCollectSlot(currentCollectSlot);
                ruleta.goTo(currentCollectSlot);
                sleep(100);
            }

            Edge right_bumper = new Edge();
            if (right_bumper.rising(gamepad1.right_bumper)) {
                shooter.spinUpTo(-1800);
            }

            Edge dpad_left = new Edge();
            if (dpad_left.rising(gamepad1.dpad_left)) {
                shooter.stopFlywheel();
            }

            boolean dpadDown = gamepad1.dpad_down;
            if (dpadDown && !lastDpadDown) {
                shooter.toggleRPM();
            }
            lastDpadDown = dpadDown;

            switch (currentState) {

                case INTAKE: {

                    boolean ballNow = sensors.ballPresent();
                    if (ballEdge.rising(ballNow)) {

                        boolean isGreen = !sensors.isPurple();
                        intake.onBallEntered(isGreen);

                        Ruleta.Slot next = ruleta.firstFreeCollectSlot();
                        if (next != null) {
                            ruleta.goTo(next);
                        }
                    }

                    if (intake.isReadyForScore()) {
                        thread2Class.setShouldAim(true);
                        ruleta.setPlan(SCORE_PLAN);
                        ruleta.moveToScore(Ruleta.Slot.C1, Ruleta.Slot.S1);
                        ruleta.moveToScore(Ruleta.Slot.C2, Ruleta.Slot.S2);
                        ruleta.moveToScore(Ruleta.Slot.C3, Ruleta.Slot.S3);
                        shooter.spinUp();
                        shotsDone = 0;
                        currentScoreSlot = null;
                        shooter.retractKicker();
                        sleep(250);
                        shootEdge.reset(false);

                        currentState = State.SCORE;
                        telemetry.update();
                    }

                    telemetry.addData("STATE", "INTAKE");
                    telemetry.addData("Balls", intake.getBallsIntaked());
                    telemetry.addData("RPM: ", shooter.showRpm());
                    telemetry.update();
                    break;
                }

                case SCORE: {
                    telemetry.update();

                    if (shooter.atSpeed()) {
                        led.setPosition(1);
                    } else {
                        led.setPosition(0);
                    }
                    sleep(200);
                    currentScoreSlot = Ruleta.Slot.S1;
                    ruleta.goTo(currentScoreSlot);

                    if (currentScoreSlot == null) {
                        currentScoreSlot = pickNextScoreSlot(ruleta);
                        if (currentScoreSlot == null) {

                            resetToIntake(shooter, intake, ruleta, rtEdge, ltEdge, ballEdge, shootEdge);
                            currentState = State.INTAKE;
                            shouldAim = false;
                            thread2Class.setShouldAim(false);
                            thread2Class.setShouldAim(false);
                            break;
                        }
                    }

                    //shoot


                    boolean shootPressed = gamepad1.cross;
                    if (shootEdge.rising(shootPressed)) {
                        intake.start();
                        for (int i = 0; i < 3 && currentScoreSlot != null; i++) {
                            shooter.pushKicker();
                            sleep(350);
                            shooter.retractKicker();
                            ruleta.popScoredBall(currentScoreSlot);
                            shotsDone++;
                            currentScoreSlot = pickNextScoreSlot(ruleta);
                            sleep(150);
                            if (currentScoreSlot != null) {
                                ruleta.goTo(currentScoreSlot);
                                sleep(250);
                            }
                        }

                        sleep(400);
                        intake.stop();

                        thread2Class.setShouldAim(false);
                        resetToIntake(shooter, intake, ruleta, rtEdge, ltEdge, ballEdge, shootEdge);
                        currentState = State.INTAKE;
                        shotsDone = 0;
                    }
                    telemetry.addData("STATE", "SCORE");
                    telemetry.addData("Vel", shooter.getVelocity());
                    telemetry.addData("AtSpeed", shooter.atSpeed());
                    telemetry.addData("TargetSlot", currentScoreSlot);
                    telemetry.addData("ShotsDone", shotsDone);
                    telemetry.addData("Ruleta", ruleta.debug());
                    telemetry.addData("Locked", thread2Class.isLocked());
                    telemetry.update();
                    break;
                }
            }

            sleep(20);
        }

        shooter.stopAll();
        intake.stop();
    }

    private Ruleta.Slot nextCollectSlot(Ruleta.Slot s) {
        if (s == Ruleta.Slot.C1) return Ruleta.Slot.C2;
        if (s == Ruleta.Slot.C2) return Ruleta.Slot.C3;
        return Ruleta.Slot.C1;
    }

    public Ruleta.Slot nextScoreSlot(Ruleta.Slot s) {
        if (s == Ruleta.Slot.S1) return Ruleta.Slot.S2;
        if (s == Ruleta.Slot.S2) return Ruleta.Slot.S3;
        return Ruleta.Slot.S1;
    }

    private static Ruleta.Slot pickNextScoreSlot(Ruleta r) {
        Ruleta.Slot s = r.nextScoreSlotFromPlan();
        if (s != null) return s;
        if (!r.isEmpty(Ruleta.Slot.S3)) return Ruleta.Slot.S3;
        if (!r.isEmpty(Ruleta.Slot.S2)) return Ruleta.Slot.S2;
        if (!r.isEmpty(Ruleta.Slot.S1)) return Ruleta.Slot.S1;
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
        ruleta.goTo(Ruleta.Slot.C1);

        rt.reset(false);
        lt.reset(false);
        ball.reset(false);
        shoot.reset(false);
    }

    public static final class Edge {
        private boolean prev = false;

        public boolean rising(boolean now) {
            boolean r = now && !prev;
            prev = now;
            return r;
        }

        void reset(boolean value) {
            prev = value;
        }
    }

    public class drivetrainThread implements Runnable {
        Telemetry telemetry;
        HardwareMap hm;
        volatile boolean isRunning = true;
        volatile boolean shouldAim = false;
        private TuretaAutoAim autoAim;
        private double TargetX = 124.87;
        private double TargetY = -62.00;
        private Shooter shooter;

        private Tureta tureta = new Tureta(hardwareMap);
        public drivetrainThread(Telemetry telemetry, HardwareMap hm, Shooter shooter) {
            this.telemetry = telemetry;
            this.hm = hm;
            this.shooter = shooter;
        }


        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(new Vector2d(0,0), Math.toRadians(0)));

        public void setShouldAim(boolean aim) {
            this.shouldAim = aim;
        }

        public boolean isLocked() {
            if (autoAim == null) return false;
            return autoAim.isAimed(0.4);
        }

        @Override
        public void run() {

            tureta.goDefault();
            autoAim = new TuretaAutoAim(hm, drive);

            autoAim.setTarget(TargetX, TargetY);

            waitForStart();

            while (!Thread.currentThread().isInterrupted()) {
                if(isRunning) {
                    drive.updatePoseEstimate();
                    double currentX = drive.pose.position.x;
                    shooter.RPMPos(currentX);

                    if (shouldAim) {
                        autoAim.setTarget(TargetX, TargetY);

                        boolean canAim = autoAim.aimToTarget();

                        if (!canAim) {
                            tureta.goDefault();
                        }
                    }else{
                        tureta.goDefault();
                    }

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    ));
                }

                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
    }
}