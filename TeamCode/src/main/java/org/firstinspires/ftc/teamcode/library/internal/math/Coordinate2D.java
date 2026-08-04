package org.firstinspires.ftc.teamcode.library.internal.math;

public class Coordinate2D {
    public double x;
    public double y;

    public Coordinate2D(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Coordinate2D negative() {
        return new Coordinate2D(-x, -y);
    }

    public void translate(Coordinate2D addend) {
        x += addend.x;
        y += addend.y;
    }

    public boolean equals(Coordinate2D check) {
        return (x == check.x && y == check.y);
    }
    public boolean equalsApproximate(Coordinate2D check, int decimalsOfAccuracy) {
        double x = check.x;
        double y = check.y;
        return (Math.abs(this.x-x)<=(1/Math.pow(10, decimalsOfAccuracy)));
    }

    public static double distanceBetweenPoints(Coordinate2D firstPoint, Coordinate2D secondPoint) {
        return (Math.sqrt(Math.pow(firstPoint.x-secondPoint.x, 2)+Math.pow(firstPoint.y-secondPoint.y, 2)));
    }
}
