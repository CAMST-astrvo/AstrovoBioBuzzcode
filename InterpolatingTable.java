package org.firstinspires.ftc.teamcode.astrovos.util;

public class InterpolatingTable {

    private final Point[] points;

    public InterpolatingTable(Point... points) {

        if (points == null || points.length == 0) {
            throw new IllegalArgumentException("InterpolatingTable needs at least one point");
        }

        this.points = points;
    }

    public double get(double x) {

        if (x <= points[0].x) {
            return points[0].y;
        }

        for (int i = 0; i < points.length - 1; i++) {
            Point low =
                    points[i];

            Point high =
                    points[i + 1];

            if (x <= high.x) {
                double t =
                        (x - low.x)
                                / (high.x - low.x);

                return low.y
                        + ((high.y - low.y) * t);
            }
        }

        return points[points.length - 1].y;
    }

    public static final class Point {

        public final double x;
        public final double y;

        public Point(
                double x,
                double y
        ) {

            this.x = x;
            this.y = y;
        }
    }
}
