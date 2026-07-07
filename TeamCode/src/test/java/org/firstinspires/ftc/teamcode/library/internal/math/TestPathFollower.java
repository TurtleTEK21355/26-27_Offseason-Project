package org.firstinspires.ftc.teamcode.library.internal.math;

import static org.junit.jupiter.api.Assertions.*;

import org.firstinspires.ftc.teamcode.library.internal.pid.PController;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

public class TestPathFollower {
    PController unused = new PController(0, 0, 0 );

    @Test
    void closestLineOnPoint() {
        ArrayList<Coordinate2D> path = new ArrayList<>();
        path.add(new Coordinate2D(0,0));
        path.add(new Coordinate2D(10,10));
        PathFollower pathFollower = new PathFollower(path, unused, unused);
        Coordinate2D robotPosition = new Coordinate2D(5,10);
        Coordinate2D closestPoint = pathFollower.getClosestPositionOnLine(robotPosition);
        assertEquals(7.5, closestPoint.x);
        assertEquals(7.5, closestPoint.y);
    }
    @Test
    void closestLineOnPointBySegmentBounds() {
        ArrayList<Coordinate2D> path = new ArrayList<>();
        path.add(new Coordinate2D(0,0));
        path.add(new Coordinate2D(10,10));
        PathFollower pathFollower = new PathFollower(path, unused, unused);
        Coordinate2D robotPosition = new Coordinate2D(5,100);
        Coordinate2D closestPoint = pathFollower.getClosestPositionOnLine(robotPosition);
        assertEquals(10, closestPoint.x);
        assertEquals(10, closestPoint.y);
    }
    @Test
    void testGetFurthestLookaheadSegmentIndex() {
        ArrayList<Coordinate2D> firstPath = new ArrayList<>();
        firstPath.add(new Coordinate2D(0,0));
        firstPath.add(new Coordinate2D(0,15));
        firstPath.add(new Coordinate2D(15,15));
        PathFollower firstPathFollower = new PathFollower(firstPath, unused, unused);
        ArrayList<Coordinate2D> secondPath = new ArrayList<>();
        secondPath.add(new Coordinate2D(0,0));
        secondPath.add(new Coordinate2D(0,7.5));
        secondPath.add(new Coordinate2D(7.5,7.5));
        PathFollower secondPathFollower = new PathFollower(secondPath, unused, unused);
        Coordinate2D robotPosition = new Coordinate2D(0,0);
        firstPathFollower.updateFurthestLookaheadSegmentIndex(robotPosition);
        secondPathFollower.updateFurthestLookaheadSegmentIndex(robotPosition);
        assertEquals(1, firstPathFollower.getFurthestLookaheadSegmentIndex());
        assertEquals(2, secondPathFollower.getFurthestLookaheadSegmentIndex());
    }
}
