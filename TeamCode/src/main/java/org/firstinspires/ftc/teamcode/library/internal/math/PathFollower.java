package org.firstinspires.ftc.teamcode.library.internal.math;

import com.qualcomm.robotcore.util.Range;

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

    public static Coordinate2D getClosestPositionOnLine(Coordinate2D lineSegmentStart, Coordinate2D lineSegmentEnd, Coordinate2D robotPosition) {
        double lineXLength = lineSegmentEnd.x - lineSegmentStart.x;
        double lineYLength = lineSegmentEnd.y - lineSegmentStart.y;
        double startToPositionXLength = robotPosition.x - lineSegmentStart.x;
        double startToPositionYLength = robotPosition.y - lineSegmentStart.y;

        double dotProductStartLineToPosition = startToPositionXLength * lineXLength + startToPositionYLength * lineYLength;
        double lineSquaredMagnitude = lineXLength * lineXLength + lineYLength * lineYLength;

        if (lineSquaredMagnitude == 0) return new Coordinate2D(lineSegmentStart.x, lineSegmentStart.y);

        double projectionFactor = Range.clip(dotProductStartLineToPosition / lineSquaredMagnitude, 0, 1);
        double closestX = lineSegmentStart.x + (lineXLength * projectionFactor);
        double closestY = lineSegmentStart.y + (lineYLength * projectionFactor);

        return new Coordinate2D(closestX, closestY);
    }

}
