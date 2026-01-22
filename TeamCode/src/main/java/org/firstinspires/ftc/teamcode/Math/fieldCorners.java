package org.firstinspires.ftc.teamcode.Math;

public class fieldCorners {
    private final double fieldWidth;
    private final double fieldHeight;

    private final double originX;
    private final double originY;

    public fieldCorners(double fieldWidth, double fieldHeight,
                        double originX, double originY) {

        this.fieldWidth = fieldWidth;
        this.fieldHeight = fieldHeight;
        this.originX = originX;
        this.originY = originY;
    }

    public static class CornerInfo {
        public final double hypotenuse;
        public final double edgeDistance;
        public final double angleRad;
        public final double angleDeg;

        public CornerInfo(double hyp, double edge, double angR) {
            this.hypotenuse = hyp;
            this.edgeDistance = edge;
            this.angleRad = angR;
            this.angleDeg = Math.toDegrees(angR);
        }
    }

    public static class Distances {
        public final CornerInfo bottomLeft;
        public final CornerInfo bottomRight;
        public final CornerInfo topLeft;
        public final CornerInfo topRight;

        public Distances(CornerInfo bl, CornerInfo br, CornerInfo tl, CornerInfo tr) {
            this.bottomLeft = bl;
            this.bottomRight = br;
            this.topLeft = tl;
            this.topRight = tr;
        }
    }

    public Distances calculate(double x, double y, double r) {

        CornerInfo bl = compute(x, y, originX, originY, r);
        CornerInfo br = compute(x, y, originX + fieldWidth, originY, r);
        CornerInfo tl = compute(x, y, originX, originY + fieldHeight, r);
        CornerInfo tr = compute(x, y, originX + fieldWidth, originY + fieldHeight, r);

        return new Distances(bl, br, tl, tr);
    }

    private CornerInfo compute(double x, double y,
                               double cx, double cy,
                               double r) {

        double dx = cx - x;
        double dy = cy - y;

        double hyp = Math.hypot(dx, dy);
        double edge = Math.max(hyp - r, 0);

        double angle = Math.atan2(dy, dx);

        return new CornerInfo(hyp, edge, angle);
    }
}
