package org.firstinspires.ftc.teamcode.Math;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "holdVel")
public class holdVel_TEST extends LinearOpMode {

    public static String MOTOR_NAME1 = "shooter1";
    public static String MOTOR_NAME2 = "shooter2";
    public static String VOLTAGE_NAME = "Control Hub";

    public static double KP = 0.085;
    public static double KI = 0.0000010;
    public static double KD = 0.00003;

    public static double KS = 0.09;
    public static double KV = 0.00050;
    public static double KA = 0;

    public static double ALPHA = 0.2;
    public static double SLEW = 4;
    public static double MAX_I = 0.2;
    public static double MAX_PWR = 1.0;

    public static double TARGET_TPS = 1500;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VelocityHoldTask task = new VelocityHoldTask(
                hardwareMap,
                KP, KI, KD,
                KS, KV, KA,
                ALPHA, SLEW, MAX_I, MAX_PWR,
                TARGET_TPS, VOLTAGE_NAME
        );

        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME1);
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME2);

        VoltageSensor battery = null;
        try { battery = hardwareMap.get(VoltageSensor.class, VOLTAGE_NAME); } catch (Exception ignored) {}

        ElapsedTime loop = new ElapsedTime();
        double lastT = 0.0;

        waitForStart();
        loop.reset();

        while (opModeIsActive()) {
            task.setTargetTps(TARGET_TPS);

            double now = loop.seconds();
            double dt = now - lastT;
            if (dt <= 0) dt = 1e-3;

            task.step();

            double vel1 = shooter1.getVelocity();
            double vel2 = shooter2.getVelocity();
            double velAvg = (vel1 + vel2) / 2.0;

            double pwr1 = shooter1.getPower();
            double pwr2 = shooter2.getPower();
            double pwrAvg = (pwr1 + pwr2) / 2.0;

            double err = TARGET_TPS - velAvg;

            double volts = (battery != null) ? battery.getVoltage() : 0.0;
            telemetry.addLine("=== Velocity Hold Tuning ===");
            telemetry.addData("Target (tps)", "%.0f", TARGET_TPS);
            telemetry.addData("Measured1 (tps)", "%.0f", vel1);
            telemetry.addData("Measured2 (tps)", "%.0f", vel2);
            telemetry.addData("MeasuredAvg (tps)", "%.0f", velAvg);
            telemetry.addData("ErrorAvg (tps)", "%.0f", err);
            telemetry.addData("Power1", "%.3f", pwr1);
            telemetry.addData("Power2", "%.3f", pwr2);
            telemetry.addData("PowerAvg", "%.3f", pwrAvg);
            telemetry.addData("Battery (V)", (battery != null) ? "%.2f" : "N/A", volts);

            telemetry.addLine("=== Gains / Limits ===");
            telemetry.addData("kP / kI / kD", "%.5f / %.7f / %.5f", KP, KI, KD);
            telemetry.addData("kS / kV / kA", "%.3f / %.6f / %.3f", KS, KV, KA);
            telemetry.addData("alpha / slew", "%.2f / %.2f", ALPHA, SLEW);
            telemetry.addData("maxI / maxPwr", "%.2f / %.2f", MAX_I, MAX_PWR);

            telemetry.addLine("=== Loop ===");
            telemetry.addData("dt (ms)", "%.1f", dt * 1000.0);
            telemetry.update();

            lastT = now;
        }

        task.stop();
    }
}
