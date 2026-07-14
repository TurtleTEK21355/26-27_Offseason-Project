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
    void testActualPath(){
        PController xController = new PController(0.05, 0, 2);
        PController yController = new PController(0.05, 0, 2);
        ArrayList<Coordinate2D> path = new ArrayList<>();
        path.add(new Coordinate2D(0,0));
        path.add(new Coordinate2D(10,0));
        path.add(new Coordinate2D(10,10));
        path.add(new Coordinate2D(0,10));
        path.add(new Coordinate2D(0,0));
        PathFollower pathFollower = new PathFollower(path, xController, yController);
    }
}
