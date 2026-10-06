package org.firstinspires.ftc.teamcode.astrovos.util;

public final class MathUtil {

    private MathUtil() {
    }

    public static double normalizeRadians(double angle) {

        while (angle > Math.PI) {
            angle -= Math.PI * 2.0;
        }

        while (angle < -Math.PI) {
            angle += Math.PI * 2.0;
        }

        return angle;
    }

    public static double normalizeDegrees(double angle) {

        while (angle > 180.0) {
            angle -= 360.0;
        }

        while (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    public static boolean isFinite(double value) {

        return !Double.isNaN(value)
                && !Double.isInfinite(value);
    }

    public static double finiteOrZero(double value) {

        return isFinite(value)
                ? value
                : 0;
    }
}
