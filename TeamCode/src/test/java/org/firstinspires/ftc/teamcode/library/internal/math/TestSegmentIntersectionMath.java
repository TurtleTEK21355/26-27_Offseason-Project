package org.firstinspires.ftc.teamcode.library.internal.math;

import static org.junit.jupiter.api.Assertions.*;


import org.junit.jupiter.api.Test;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;

public class TestSegmentIntersectionMath {
    @Test
    void circleHas2Intersections() {
        double circleRadius = 9;
        Coordinate2D lineSegmentStart = new Coordinate2D(0, 0);
        Coordinate2D lineSegmentEnd = new Coordinate2D(40, 50);
        Coordinate2D circleCenter = new Coordinate2D(20, 30);
        List<Coordinate2D> intersections = SegmentIntersectionMath.intersectionsLineSegmentAndCircle(lineSegmentStart, lineSegmentEnd, circleCenter, circleRadius);
        assertEquals(2, intersections.size());
        List<Coordinate2D> validCoords = new ArrayList<>();
        validCoords.add(new Coordinate2D(17.17, 21.46));
        validCoords.add(new Coordinate2D(27.71, 34.64));
        boolean firstCoordinateCheck = false;
        boolean secondCoordinateCheck = false;
        for (Coordinate2D intersection : intersections) {
            double intersectionX = roundToHundredths(intersection.x);
            double intersectionY = roundToHundredths(intersection.y);

            if (validCoords.get(0).x == intersectionX && validCoords.get(0).y == intersectionY)
                firstCoordinateCheck = true;
            else if (validCoords.get(1).x == intersectionX && validCoords.get(1).y == intersectionY)
                secondCoordinateCheck = true;
        }
        assertTrue(firstCoordinateCheck && secondCoordinateCheck);

    }

    @Test
    void circleHas1Intersection() {
        double circleRadius = 9;
        Coordinate2D lineSegmentStart = new Coordinate2D(-10, 0);
        Coordinate2D lineSegmentEnd =   new Coordinate2D(10, 0);
        Coordinate2D circleCenter =     new Coordinate2D(0, 9);
        List<Coordinate2D> intersections = SegmentIntersectionMath.intersectionsLineSegmentAndCircle(lineSegmentStart, lineSegmentEnd, circleCenter, circleRadius);
        assertEquals(1, intersections.size());
        assertEquals(0, roundToHundredths(intersections.get(0).x));
        assertEquals(0, roundToHundredths(intersections.get(0).y));
    }

    @Test
    void circleHasNoIntersections() {
        double circleRadius = 9;
        Coordinate2D lineSegmentStart = new Coordinate2D(10, 5);
        Coordinate2D lineSegmentEnd = new Coordinate2D(40, -50);
        Coordinate2D circleCenter = new Coordinate2D(20, 60);
        List<Coordinate2D> intersections = SegmentIntersectionMath.intersectionsLineSegmentAndCircle(lineSegmentStart, lineSegmentEnd, circleCenter, circleRadius);
        assertEquals(0, intersections.size());
    }

    @Test
    void circleHas1IntersectionDueToSegmentBounds() {
        double circleRadius = 9;
        Coordinate2D lineSegmentStart = new Coordinate2D(-3, -3);
        Coordinate2D lineSegmentEnd = new Coordinate2D(8, 8);
        Coordinate2D circleCenter = new Coordinate2D(0, 0);
        List<Coordinate2D> intersections = SegmentIntersectionMath.intersectionsLineSegmentAndCircle(lineSegmentStart, lineSegmentEnd, circleCenter, circleRadius);
        assertEquals(1, intersections.size());
        assertEquals(6.36, roundToHundredths(intersections.get(0).x));
        assertEquals(6.36, roundToHundredths(intersections.get(0).y));
    }

    @Test
    void circleHasNoIntersectionsDueToSegmentBoundsOutside() {
        double circleRadius = 9;
        Coordinate2D lineSegmentStart = new Coordinate2D(6.5, -8.5);
        Coordinate2D lineSegmentEnd = new Coordinate2D(19, 2);
        Coordinate2D circleCenter = new Coordinate2D(27.8, 7.7);
        List<Coordinate2D> intersections = SegmentIntersectionMath.intersectionsLineSegmentAndCircle(lineSegmentStart, lineSegmentEnd, circleCenter, circleRadius);
        assertEquals(0, intersections.size());
    }

    @Test
    void circleHasNoIntersectionsDueToSegmentBoundsInside() {
        double circleRadius = 9;
        Coordinate2D lineSegmentStart = new Coordinate2D(-10.5, -3.5);
        Coordinate2D lineSegmentEnd = new Coordinate2D(-8.5, 8);
        Coordinate2D circleCenter = new Coordinate2D(-7.2, 2.5);
        List<Coordinate2D> intersections = SegmentIntersectionMath.intersectionsLineSegmentAndCircle(lineSegmentStart, lineSegmentEnd, circleCenter, circleRadius);
        assertEquals(0, intersections.size());
    }

    @Test
    void circleHas1IntersectionDueToProximity() {
        double circleRadius = 9;
        Coordinate2D lineSegmentStart = new Coordinate2D(0, 0);
        Coordinate2D lineSegmentEnd = new Coordinate2D(24, 0);
        Coordinate2D circleCenter = new Coordinate2D(12, 8.99);
        List<Coordinate2D> intersections = SegmentIntersectionMath.intersectionsLineSegmentAndCircle(lineSegmentStart, lineSegmentEnd, circleCenter, circleRadius);
        assertEquals(1, intersections.size());
        assertTrue(roundToHundredths(intersections.get(0).x) == 11.58 || roundToHundredths(intersections.get(0).x) == 12.42);
        assertEquals(0, roundToHundredths(intersections.get(0).y));
    }

    @Test
    void pointIsFurtherAlongLineSegment() {
        Coordinate2D lineSegmentEnd = new Coordinate2D(10, 10);
        ArrayList<Coordinate2D> intersections = new ArrayList<>();
        intersections.add(new Coordinate2D(2, 2));
        intersections.add(new Coordinate2D(40, 0));
        intersections.add(new Coordinate2D(12, 12.001));
        intersections.add(new Coordinate2D(8,8));
        Coordinate2D closerIntersection = SegmentIntersectionMath.pointFurthestAlongLineSegment(lineSegmentEnd, intersections);
        assertNotNull(closerIntersection);
        assertTrue(closerIntersection.x == 8 && closerIntersection.y == 8);
    }

    @Test
    void inputDoesNotChange() {
        double circleRadius = 9;
        Coordinate2D lineSegmentStart = new Coordinate2D(-10, 0);
        Coordinate2D lineSegmentEnd =   new Coordinate2D(10, 0);
        Coordinate2D circleCenter =     new Coordinate2D(0, 9);
        Coordinate2D lineSegmentStartReference = new Coordinate2D(-10, 0);
        Coordinate2D lineSegmentEndReference =   new Coordinate2D(10, 0);
        Coordinate2D circleCenterReference =     new Coordinate2D(0, 9);
        SegmentIntersectionMath.intersectionsLineSegmentAndCircle(lineSegmentStart, lineSegmentEnd, circleCenter, circleRadius);
        assertTrue(lineSegmentStartReference.equals(lineSegmentStart));
        assertTrue(lineSegmentEndReference.equals(lineSegmentEnd));
        assertTrue(circleCenterReference.equals(circleCenter));

    }

//        -3.71 -0.05
    @Test
    void specificValueTest() {
        double circleRadius = 9;
        Coordinate2D lineSegmentStart = new Coordinate2D(0, 0);
        Coordinate2D lineSegmentEnd = new Coordinate2D(-24, 0);
        Coordinate2D circleCenter = new Coordinate2D(-3.71, -0.05);
        List<Coordinate2D> intersections = SegmentIntersectionMath.intersectionsLineSegmentAndCircle(lineSegmentStart, lineSegmentEnd, circleCenter, circleRadius);
        for (int i = 0; i < intersections.size(); i++) {
            System.out.println("("+intersections.get(i).x+", "+intersections.get(i).y+")");
        }
    }

    double roundToHundredths(double input) {
        return BigDecimal.valueOf(input).setScale(2, RoundingMode.HALF_UP).doubleValue();
    }
}
