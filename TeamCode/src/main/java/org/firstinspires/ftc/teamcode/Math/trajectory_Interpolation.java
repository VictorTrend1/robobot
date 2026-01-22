package org.firstinspires.ftc.teamcode.Math;

public class trajectory_Interpolation {
    public enum Mode {
        LINEAR,
        MONOTONIC_CUBIC
    }

    public static final double MIN_DIST = 40.0;
    public static final double MAX_DIST = 120.0;

    public static final double MIN_SERVO = 0.20;
    public static final double MAX_SERVO = 0.70;

    public static final Mode MODE = Mode.MONOTONIC_CUBIC;

    public static final double[] D = {40.0, 60.0, 80.0, 100.0, 120.0};
    public static final double[] S = {0.28, 0.34, 0.41, 0.48, 0.55};

    private static final double[] M = computeSlopes();

    public static double getServoForDistance(double distance) {
        if (D.length < 2) return clamp(MIN_SERVO, MIN_SERVO, MAX_SERVO);
        double d = clamp(distance, MIN_DIST, MAX_DIST);
        double servo;
        if (MODE == Mode.MONOTONIC_CUBIC && M.length == D.length) {
            servo = interpolateMonotoneCubic(d);
        } else {
            servo = interpolateLinear(d);
        }
        return clamp(servo, MIN_SERVO, MAX_SERVO);
    }

    private static double interpolateLinear(double x) {
        int n = D.length;
        if (x <= D[0]) return S[0];
        if (x >= D[n - 1]) return S[n - 1];
        for (int i = 0; i < n - 1; i++) {
            double x0 = D[i];
            double x1 = D[i + 1];
            if (x >= x0 && x <= x1) {
                double y0 = S[i];
                double y1 = S[i + 1];
                double t = (x - x0) / (x1 - x0);
                return y0 + t * (y1 - y0);
            }
        }
        return S[n - 1];
    }

    private static double[] computeSlopes() {
        int n = D.length;
        if (n < 2) return new double[0];
        double[] delta = new double[n - 1];
        double[] m = new double[n];
        for (int i = 0; i < n - 1; i++) {
            double h = D[i + 1] - D[i];
            delta[i] = (S[i + 1] - S[i]) / h;
        }
        m[0] = delta[0];
        m[n - 1] = delta[n - 2];
        for (int i = 1; i < n - 1; i++) {
            if (delta[i - 1] * delta[i] <= 0.0) {
                m[i] = 0.0;
            } else {
                double w1 = 2.0 * (D[i + 1] - D[i]);
                double w2 = 2.0 * (D[i] - D[i - 1]);
                m[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
            }
        }
        for (int i = 0; i < n - 1; i++) {
            if (delta[i] == 0.0) {
                m[i] = 0.0;
                m[i + 1] = 0.0;
            } else {
                double a = m[i] / delta[i];
                double b = m[i + 1] / delta[i];
                double sum = a * a + b * b;
                if (sum > 9.0) {
                    double t = 3.0 / Math.sqrt(sum);
                    m[i] = t * a * delta[i];
                    m[i + 1] = t * b * delta[i];
                }
            }
        }
        return m;
    }

    private static double interpolateMonotoneCubic(double x) {
        int n = D.length;
        if (x <= D[0]) return S[0];
        if (x >= D[n - 1]) return S[n - 1];
        int i = 0;
        for (; i < n - 1; i++) {
            if (x <= D[i + 1]) break;
        }
        double x0 = D[i];
        double x1 = D[i + 1];
        double y0 = S[i];
        double y1 = S[i + 1];
        double m0 = M[i];
        double m1 = M[i + 1];
        double h = x1 - x0;
        double t = (x - x0) / h;
        double t2 = t * t;
        double t3 = t2 * t;
        double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        double h10 = t3 - 2.0 * t2 + t;
        double h01 = -2.0 * t3 + 3.0 * t2;
        double h11 = t3 - t2;
        return h00 * y0 + h10 * h * m0 + h01 * y1 + h11 * h * m1;
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}

