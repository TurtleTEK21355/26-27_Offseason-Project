package org.firstinspires.ftc.teamcode.library.internal.math;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class TestPathFollower {
    @Test
    void closestLineOnPoint() {
        Coordinate2D lineSegmentStart = new Coordinate2D(0,0);
        Coordinate2D lineSegmentEnd = new Coordinate2D(10,10);
        Coordinate2D robotPosition = new Coordinate2D(5,10);
        Coordinate2D closestPoint = PathFollower.getClosestPointOnLine(lineSegmentStart, lineSegmentEnd, robotPosition);
        assertEquals(7.5, closestPoint.x);
        assertEquals(7.5, closestPoint.y);
    }
    @Test
    void closestLineOnPointBySegmentBounds() {
        Coordinate2D lineSegmentStart = new Coordinate2D(0,0);
        Coordinate2D lineSegmentEnd = new Coordinate2D(10,10);
        Coordinate2D robotPosition = new Coordinate2D(5,100);
        Coordinate2D closestPoint = PathFollower.getClosestPointOnLine(lineSegmentStart, lineSegmentEnd, robotPosition);
        assertEquals(10, closestPoint.x);
        assertEquals(10, closestPoint.y);
    }
}
