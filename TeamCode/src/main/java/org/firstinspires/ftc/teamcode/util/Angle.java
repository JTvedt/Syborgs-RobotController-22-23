package org.firstinspires.ftc.teamcode.util;

public class Angle {
    public static enum Unit {
        DEGREES,
        RADIANS
    }

    public static double radians(double degrees) {
        return degrees * Math.PI/180;
    }

    public static double degrees(double radians) {
        return radians * 180/Math.PI;
    }

    public static double boundAngle(double angle) {
        if (angle > Math.PI) return angle - 2 * Math.PI;
        if (angle < -Math.PI) return angle + 2 * Math.PI;
        return angle;
    }

    public static double roundAngle(double angle) {
        return Math.round(angle * 2/Math.PI) * Math.PI/2;
    }
}
