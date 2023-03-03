package org.firstinspires.ftc.teamcode.util;

public class Angle {
    public static enum Unit {
        DEGREES,
        RADIANS
    }

    public static double toRadians(double degrees) {
        return degrees * Math.PI/180;
    }

    public static double toDegrees(double radians) {
        return radians * 180/Math.PI;
    }

    public static double bound(double angle) {
        if (angle > Math.PI) return angle - 2 * Math.PI;
        if (angle < -Math.PI) return angle + 2 * Math.PI;
        return angle;
    }

    public static double round(double angle, int increments) {
        return Math.round(angle * (increments/2d)/Math.PI) * Math.PI/(increments/2d);
    }

    public static double round(double angle) {
        return round(angle, 4);
    }

    public static double difference(double startAngle, double targetAngle) {
        double rawDistance = bound(targetAngle) - startAngle;
        if (Math.abs(rawDistance) > Math.PI)
            return rawDistance - Math.signum(rawDistance) * 2 * Math.PI;
        else return rawDistance;
    }
}
