package org.firstinspires.ftc.teamcode.library.internal;

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

//    To add if needed: reflect(double axis), rotate(double degrees), stretch(Double axis, double amount)

    public static double distanceBetweenPoints(Coordinate2D firstPoint, Coordinate2D secondPoint) {
        return (Math.sqrt(Math.pow(firstPoint.x-secondPoint.x, 2)+Math.pow(firstPoint.y-secondPoint.y, 2)));
    }
}
