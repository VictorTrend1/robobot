package org.firstinspires.ftc.teamcode.Math;

public final class trajectory_Interpolation {

    public static final double G = 9.80665;

    public static final double THETA_DEG = 35.0;
    public static final double DELTA_H = 0.0;

    public static final double DIST_BIAS = 0.0;

    public static final double D1 = 31.4961;
    public static final double D2 = 39.3701;
    public static final double D3 = 78.7402;
    public static final double D4 = 118.11;
    public static final double D5 = 156.0;
    public static final double D6 = 250.00;

    public static final double RPM1 = 1350.0;
    public static final double RPM2 = 1200.0;
    public static final double RPM3 = 1100.0;
    public static final double RPM4 = 1150.0;
    public static final double RPM5 = 1250.0;
    public static final double RPM6 = 1450.0;


    private static final double K_RPM_PER_MPS = fitK();

    private trajectory_Interpolation() {}

    public static double rpmForDistance(double distance) {
        double v = requiredExitSpeed(distance + DIST_BIAS);
        if (v <= 0.0) return 0.0;
        return K_RPM_PER_MPS * v;
    }

    private static double requiredExitSpeed(double d) {
        if (d <= 0.0) return -1.0;

        double theta = Math.toRadians(THETA_DEG);
        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double denom = 2.0 * cos * cos * (d * tan - DELTA_H);
        if (denom <= 1e-9) return -1.0;

        return Math.sqrt((G * d * d) / denom);
    }

    private static double fitK() {
        double[] ds = {D1, D2, D3, D4 ,D5,D6};
        double[] rs = {RPM1, RPM2, RPM3, RPM4,RPM5,RPM6};

        double num = 0.0;
        double den = 0.0;

        for (int i = 0; i < 4; i++) {
            double v = requiredExitSpeed(ds[i] + DIST_BIAS);
            if (v > 1e-9) {
                num += rs[i] * v;
                den += v * v;
            }
        }
        return (den > 1e-9) ? (num / den) : 0.0;
    }
}
