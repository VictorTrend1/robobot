package org.firstinspires.ftc.teamcode.temp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name = "Shooter PIDF Tuner", group = "Tuning")
public class ShooterPIDFTuner extends OpMode {

    public static double kP = 350.0;
    public static double kI = 0.0;
    public static double kD = 1.0;
    public static double kF = 13.0;
    public static double TARGET_VELOCITY = 1660.0;

    private static final double DEFAULT_P      = 350.0;
    private static final double DEFAULT_I      = 0.0;
    private static final double DEFAULT_D      = 1.0;
    private static final double DEFAULT_F      = 13.0;
    private static final double DEFAULT_TARGET = 1660.0;

    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;

    private FtcDashboard dashboard;

    private boolean flywheelRunning = false;

    private double lastP = -1, lastI = -1, lastD = -1, lastF = -1;

    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevB = false;

    private String snapshot = "Press X to capture snapshot";

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        flywheel  = hardwareMap.get(DcMotorEx.class, "shooter1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pushPIDF();
        flywheel.setVelocity(0);
        flywheel2.setPower(0);

        telemetry.addLine("Ready. A = toggle flywheel | B = reset | X = snapshot");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean currA = gamepad1.a;
        if (currA && !prevA) {
            flywheelRunning = !flywheelRunning;
            if (flywheelRunning) {
                flywheel.setVelocity(TARGET_VELOCITY);
                flywheel2.setPower(flywheel.getPower());
            } else {
                flywheel.setVelocity(0);
                flywheel2.setPower(0);
            }
        }
        prevA = currA;

        boolean currB = gamepad1.b;
        if (currB && !prevB) {
            kP = DEFAULT_P;
            kI = DEFAULT_I;
            kD = DEFAULT_D;
            kF = DEFAULT_F;
            TARGET_VELOCITY = DEFAULT_TARGET;
            pushPIDF();
            if (flywheelRunning) {
                flywheel.setVelocity(TARGET_VELOCITY);
                flywheel2.setPower(flywheel.getPower());
            }
        }
        prevB = currB;

        boolean currX = gamepad1.x;
        if (currX && !prevX) {
            snapshot = String.format(
                    "P=%.1f  I=%.2f  D=%.2f  F=%.2f  | Target=%.0f  Actual=%.1f",
                    kP, kI, kD, kF, TARGET_VELOCITY, flywheel.getVelocity()
            );
        }
        prevX = currX;

        if (pidfChanged()) {
            pushPIDF();
            if (flywheelRunning) {
                flywheel.setVelocity(TARGET_VELOCITY);
                flywheel2.setPower(flywheel.getPower());
            }
        }

        double currentVelocity = flywheel.getVelocity();
        double error = TARGET_VELOCITY - currentVelocity;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Velocity", TARGET_VELOCITY);
        packet.put("Current Velocity", currentVelocity);
        packet.put("Error", error);
        packet.put("kP", kP);
        packet.put("kI", kI);
        packet.put("kD", kD);
        packet.put("kF", kF);
        packet.put("Flywheel Running", flywheelRunning);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Target Velocity", TARGET_VELOCITY);
        telemetry.addData("Current Velocity", "%.1f", currentVelocity);
        telemetry.addData("Error", "%.1f", error);
        telemetry.addData("PIDF", "P=%.1f  I=%.2f  D=%.2f  F=%.2f", kP, kI, kD, kF);
        telemetry.addData("Flywheel", flywheelRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Snapshot", snapshot);
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.setVelocity(0);
        flywheel2.setPower(0);
    }

    private boolean pidfChanged() {
        return kP != lastP || kI != lastI || kD != lastD || kF != lastF;
    }

    private void pushPIDF() {
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );
        lastP = kP;
        lastI = kI;
        lastD = kD;
        lastF = kF;
    }
}