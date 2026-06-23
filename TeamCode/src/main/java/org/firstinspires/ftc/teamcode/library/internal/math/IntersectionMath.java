package org.firstinspires.ftc.teamcode.library.internal.math;

import org.firstinspires.ftc.teamcode.library.internal.Coordinate2D;

import java.util.ArrayList;
import java.util.List;

public class IntersectionMath {
    public static List<Coordinate2D> intersectionsLineSegmentAndCircle(Coordinate2D lineSegmentStart, Coordinate2D lineSegmentEnd, Coordinate2D circleCenter, double feedforwardRadius) {
        lineSegmentStart.translate(circleCenter.negative());
        lineSegmentEnd.translate(circleCenter.negative());

        double dX = lineSegmentEnd.x - lineSegmentStart.x;
        double dY = lineSegmentEnd.y - lineSegmentStart.y;
        double segmentLength = Math.sqrt((dX * dX) + (dY * dY));
        double determinant = (lineSegmentStart.x * lineSegmentEnd.y) - (lineSegmentEnd.x * lineSegmentStart.y);
        double sign;
        if (dY < 0) sign = -1;
        else sign = 1;

        double discriminant = (feedforwardRadius * feedforwardRadius * segmentLength * segmentLength) - (determinant * determinant);

        ArrayList<Coordinate2D> intersections = new ArrayList<>();

        if (discriminant >= 0) {
            double yOffset = Math.abs(dY) * Math.sqrt(discriminant);

            intersections.add( new Coordinate2D(((determinant * dY) + (sign * dX * Math.sqrt(discriminant))) / (segmentLength * segmentLength),
                                                ((-determinant * dX) + yOffset) / (segmentLength * segmentLength)));

            intersections.add( new Coordinate2D(((determinant * dY) - (sign * dX * Math.sqrt(discriminant))) / (segmentLength * segmentLength),
                                                ((-determinant * dX) - yOffset) / (segmentLength * segmentLength)));

            intersections.removeIf(intersection ->
                    intersection.x < Math.min(lineSegmentStart.x, lineSegmentEnd.x) || intersection.x > Math.max(lineSegmentStart.x, lineSegmentEnd.x)
                 || intersection.y < Math.min(lineSegmentStart.y, lineSegmentEnd.y) || intersection.y > Math.max(lineSegmentStart.y, lineSegmentEnd.y));
            intersections.forEach(intersection -> intersection.translate(circleCenter));
            if (intersections.size() == 2) {
                if (Coordinate2D.distanceBetweenPoints(intersections.get(0), intersections.get(1)) <= MathConstants.intersectionMaxMergeDistance) {
                    intersections.remove(1);
                }
            }
        }
        return intersections;
    }
    public static Coordinate2D pointFurthestAlongLineSegment(Coordinate2D lineSegmentEnd, List<Coordinate2D> intersections) {
        Double smallestDistance = null;
        Integer smallestDistanceIndex = null;

        for (int i = 0; i < intersections.size(); i++) {
            double distance = Coordinate2D.distanceBetweenPoints(intersections.get(i), lineSegmentEnd);

            if (smallestDistance == null || distance < smallestDistance) {
                smallestDistance = distance;
                smallestDistanceIndex = i;
            }
        }

        if (smallestDistanceIndex != null) {
            return intersections.get(smallestDistanceIndex);
        } else {
            return null;
        }
    }
}