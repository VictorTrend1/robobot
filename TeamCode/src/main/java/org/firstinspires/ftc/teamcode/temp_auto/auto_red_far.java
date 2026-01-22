package org.firstinspires.ftc.teamcode.temp_auto;

import static org.firstinspires.ftc.teamcode.recunoastere.Limelight.getAprilTagId;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpMode.Teleop;
import org.firstinspires.ftc.teamcode.temp.Teleop_v2;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.systems.Inaltime;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.RampSensors;
import org.firstinspires.ftc.teamcode.systems.Ruleta;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Tureta;

import java.util.Arrays;

@Autonomous(name = "auto_red_far")
public class auto_red_far extends LinearOpMode {
    private final Ruleta.Plan3 SCORE_PLAN = Ruleta.Plan3.PGP;
    Limelight3A limelight;

    String caz;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        VelConstraint vel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(120),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint acc = new ProfileAccelConstraint(-50, 80);

        Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, start);

        Teleop.Edge ballEdge = new Teleop_v2.Edge();

        Ruleta ruleta = new Ruleta(hardwareMap);
        Tureta tureta = new Tureta(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        intake.setRuleta(ruleta);

        Shooter shooter = new Shooter(hardwareMap);

        RampSensors sensors = new RampSensors(hardwareMap);

        Inaltime inaltime = new Inaltime(hardwareMap);


        shooter.stopAll();
        intake.resetForIntake();
        shooter.safeForRuletaRotate();
        ruleta.goTo(Ruleta.Slot.S1);

        waitForStart();
        while (opModeInInit()){

            LLResult result = limelight.getLatestResult();
            String tagId = getAprilTagId(result);

            telemetry.addData("AprilTag ID", tagId);
            telemetry.update();
        }

        // 1) MERGI LA POZITIA DE SHOOT
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .afterTime(0.0, () -> {
                            tureta.goDefault();
                            shooter.spinUpTo(1700);
                        })
                        .strafeToLinearHeading(new Vector2d(15, -2), Math.toRadians(-41), vel, acc)
                        .afterTime(0.1, ()->{ new Thread(()->{

                            while (!shooter.atSpeedTo(1650)){}
                            ruleta.goTo(Ruleta.Slot.S1);

                            kick_ball(shooter);
                            sleep(450);

                            ruleta.goTo(Ruleta.Slot.S2);
                            while (!shooter.atSpeedTo(1650)){}

                            sleep(300);
                            kick_ball(shooter);
                            sleep(300);

                            ruleta.goTo(Ruleta.Slot.S3);
                            while (!shooter.atSpeedTo(1650)){}

                            sleep(300);
                            kick_ball(shooter);

                        }).start();})
                        .build()
        );
        sleep(4000); // Wait for shooting thread to finish

        // 2) MERGI LA ZONA DE COLECTARE
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d (drive.pose.position.x,drive.pose.position.y),drive.pose.heading.toDouble()))
                        .afterTime(0.0, () -> {
                            new Thread(() -> {
                                tureta.goDefault();
                                shooter.stopFlywheel();
                                ruleta.goTo(Ruleta.Slot.C1);
                            }).start();
                        })
                        .strafeToLinearHeading(new Vector2d(26.5, -27.2), Math.toRadians(-92), vel, acc)
                        .build()
        );

        sleep(700);

        // 3) COBORI SI MAI MULT, PORNESTI INTAKE
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d (drive.pose.position.x,drive.pose.position.y),drive.pose.heading.toDouble()))
                        .afterTime(0.0, () -> {
                            tureta.goDefault();
                            intake.start();
                            collectForMs(intake, shooter, ruleta, sensors, ballEdge, 1500);
                        })
                        .splineTo(new Vector2d(drive.pose.position.x+12 , drive.pose.position.y), drive.pose.heading.toDouble())
                        .build()
        );


        // 4) INAPOI LA SHOOT
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(new Vector2d (drive.pose.position.x,drive.pose.position.y),drive.pose.heading.toDouble()))
                        .afterTime(0.0, () -> {
                            tureta.goDefault();
                            shooter.spinUpTo(1700);
                            ruleta.goTo(Ruleta.Slot.S1);
                        }) // -36.7 , -22.8
                        .strafeToLinearHeading(new Vector2d(-24, -2), Math.toRadians(-41), vel, acc)
                        .afterTime(0.1, ()->{ new Thread(()->{

                            while (!shooter.atSpeedTo(1650)){}
                            ruleta.goTo(Ruleta.Slot.S1);

                            kick_ball(shooter);
                            sleep(450);

                            ruleta.goTo(Ruleta.Slot.S2);
                            while (!shooter.atSpeedTo(1650)){}

                            sleep(300);
                            kick_ball(shooter);
                            sleep(300);

                            ruleta.goTo(Ruleta.Slot.S3);
                            while (!shooter.atSpeedTo(1650)){}

                            sleep(300);
                            kick_ball(shooter);

                        }).start();})
                        .build()
        );

        sleep(4000);

        shooter.stopFlywheel();
        intake.stop();
    }

    private void kick_ball(Shooter shooter) {
        shooter.pushKicker();
        sleep(200);
        shooter.retractKicker();
        sleep(200);
        shooter.pushKicker();
        sleep(200);
        shooter.retractKicker();
    }

    private void collectForMs(Intake intake, Shooter shooter, Ruleta ruleta, RampSensors sensors, Teleop.Edge ballEdge, long ms) {
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < ms) {
            boolean ballNow = sensors.ballPresent();
            if (ballEdge.rising(ballNow)) {
                boolean isGreen = !sensors.isPurple();
                intake.onBallEntered(isGreen);
                Ruleta.Slot next = ruleta.firstFreeCollectSlot();
                if (next != null) {
                    shooter.safeForRuletaRotate();
                    sleep(200);
                    ruleta.goTo(next);
                }
            }
            idle();
        }
    }
}