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
        Coordinate2D lineLengths = new Coordinate2D(lineSegmentEnd.x - lineSegmentStart.x,
                                                   lineSegmentEnd.y - lineSegmentStart.y);
        Coordinate2D startToPositionLengths = new Coordinate2D(robotPosition.x - lineSegmentStart.x,
                                                               robotPosition.y - lineSegmentStart.y);

        double dotProductStartLineToPosition = startToPositionLengths.x * lineLengths.x + startToPositionLengths.y * lineLengths.y;
        double lineSquaredMagnitude = lineLengths.x * lineLengths.x + lineLengths.y * lineLengths.y;
        if (lineSquaredMagnitude == 0) return new Coordinate2D(lineSegmentStart.x, lineSegmentStart.y);

        double projectionFactor = Range.clip(dotProductStartLineToPosition / lineSquaredMagnitude, 0, 1);
        return new Coordinate2D(lineSegmentStart.x + (lineLengths.x * projectionFactor),
                                lineSegmentStart.y + (lineLengths.y * projectionFactor));
    }

}
