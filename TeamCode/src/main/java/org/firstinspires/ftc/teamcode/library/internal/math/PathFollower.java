package org.firstinspires.ftc.teamcode.library.internal.math;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.library.internal.Pose2D;
import org.firstinspires.ftc.teamcode.library.internal.pid.PController;
import org.firstinspires.ftc.teamcode.library.internal.telemetry.TelemetryPasser;

import java.util.List;

public class PathFollower {
    private final List<Coordinate2D> path;
    PController xController;
    PController yController;

    public PathFollower(List<Coordinate2D> path, PController xController, PController yController) {
        this.path = path;
        this.xController = xController;
        this.yController = yController;
    }
    public Coordinate2D getClosestPositionOnLine(Coordinate2D robotPosition) {
        Coordinate2D lineSegmentStart = path.get(0);
        Coordinate2D lineSegmentEnd = path.get(1);
        Coordinate2D lineLengths = new Coordinate2D(lineSegmentEnd.x - lineSegmentStart.x,
                                                   lineSegmentEnd.y - lineSegmentStart.y);
        Coordinate2D startToPositionLengths = new Coordinate2D(robotPosition.x - lineSegmentStart.x,
                                                               robotPosition.y - lineSegmentStart.y);
        double dotProductStartLineToPosition = startToPositionLengths.x * lineLengths.x + startToPositionLengths.y * lineLengths.y;
        double lineSquaredMagnitude = lineLengths.x * lineLengths.x + lineLengths.y * lineLengths.y;
        if (lineSquaredMagnitude == 0) return new Coordinate2D(lineSegmentStart.x, lineSegmentStart.y);

        double projectionFactor = Range.clip(dotProductStartLineToPosition / lineSquaredMagnitude, 0, 1);
        Coordinate2D closestPosition = new Coordinate2D(lineSegmentStart.x + (lineLengths.x * projectionFactor),
                                                        lineSegmentStart.y + (lineLengths.y * projectionFactor));
        if (closestPosition.equals(lineSegmentEnd) && path.size() > 2) {
            path.remove(0);
            return getClosestPositionOnLine(robotPosition);
        } else return closestPosition;
    }

    public void updatePControllerTarget(Coordinate2D robotPosition) {
        double distanceAlreadyCrossed = Coordinate2D.distanceBetweenPoints(getClosestPositionOnLine(robotPosition), path.get(0));
        double totalDistance = -distanceAlreadyCrossed;
        int furthestLookaheadSegmentIndex = 0;
        while (totalDistance < MathConstants.pathLookaheadDistance && path.size()-furthestLookaheadSegmentIndex > 1) {
            totalDistance += Coordinate2D.distanceBetweenPoints(path.get(furthestLookaheadSegmentIndex), path.get(furthestLookaheadSegmentIndex+1));
            if (totalDistance < MathConstants.pathLookaheadDistance) {
                furthestLookaheadSegmentIndex++;
            }
        }
        Coordinate2D result = null;
        if(furthestLookaheadSegmentIndex != 0) {
            for (int i = 0; i < furthestLookaheadSegmentIndex; i++) {
                List<Coordinate2D> intersections = SegmentIntersectionMath.intersectionsLineSegmentAndCircle(path.get(i), path.get(i + 1), robotPosition, 9);
                if (!intersections.isEmpty()) {
                    result = SegmentIntersectionMath.pointFurthestAlongLineSegment(path.get(i + 1), intersections);
                }
            }
            if (result == null) {
                result = path.get(0);
                TelemetryPasser.telemetry.addData("Found Intersections:", false);
            }
        } else result = path.get(0);
        xController.setTargetPosition(result.x);
        yController.setTargetPosition(result.y);
        TelemetryPasser.telemetry.addData("Target Position:", "(" + result.x + ", " + result.y + ")");
    }

    public void updatePControllerTarget(Pose2D robotPosition) {
        updatePControllerTarget(new Coordinate2D(robotPosition.x, robotPosition.y));
    }

    public double getXSpeed(double robotXPosition) {return xController.calculate(robotXPosition);}
    public double getYSpeed(double robotYPosition) {return yController.calculate(robotYPosition);}
}
