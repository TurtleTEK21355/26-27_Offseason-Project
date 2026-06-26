package org.firstinspires.ftc.teamcode.library.internal.math;

import java.util.ArrayList;
import java.util.List;

public class PathFollower {
    private int currentSegmentIndex;
    private List<Coordinate2D> path;

    public PathFollower(List<Coordinate2D> path) {
        this.path = path;
        currentSegmentIndex = 0;
    }
    public PathFollower(List<Coordinate2D> path, int startingIndex) {
        this.path = path;
        currentSegmentIndex = startingIndex;
    }

    public static Coordinate2D getClosestPointOnLine(Coordinate2D lineSegmentStart, Coordinate2D lineSegmentEnd, Coordinate2D robotPosition) {
        double abX = lineSegmentEnd.x - lineSegmentStart.x;
        double abY = lineSegmentEnd.y - lineSegmentStart.y;
        double apX = robotPosition.x - lineSegmentStart.x;
        double apY = robotPosition.y - lineSegmentStart.y;

        double dotAP_AB = apX * abX + apY * abY;
        double dotAB_AB = abX * abX + abY * abY;

        if (dotAB_AB == 0) {
            return new Coordinate2D(lineSegmentStart.x, lineSegmentStart.y);
        }

        double t = dotAP_AB / dotAB_AB;

         t = Math.max(0f, Math.min(1f, t));

        double closestX = lineSegmentStart.x + t * abX;
        double closestY = lineSegmentStart.y + t * abY;

        return new Coordinate2D(closestX, closestY);
    }

}
