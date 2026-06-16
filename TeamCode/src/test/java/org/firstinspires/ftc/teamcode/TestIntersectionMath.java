package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;


import org.junit.jupiter.api.Test;

public class TestIntersectionMath {

    double centerX = 9.3;
    double centerY = 31.6;
    double lineStartX = 29;
    double lineStartY = 41;
    double lineEndX = 0.5;
    double lineEndY = 14;
    double feedforwardRadius = 9;
    double discriminantTolerance = 5;

    @Test
    void intersectionMath() {
        lineStartX -= centerX;
        lineStartY -= centerY;
        lineEndX -= centerX;
        lineEndY -= centerY;

        double dX = lineEndX-lineStartX;
        double dY = lineEndY-lineStartY;
        double segmentLength = Math.sqrt((dX*dX)+(dY*dY));
        double determinant = (lineStartX*lineEndY)-(lineEndX*lineStartY);
        double sign;
        if (dY<0) sign = -1;
        else sign = 1;

        double discriminant = (feedforwardRadius*feedforwardRadius*segmentLength*segmentLength)-(determinant*determinant);

        Double firstIntersectionX = null;
        Double secondIntersectionX = null;
        Double firstIntersectionY = null;
        Double secondIntersectionY = null;

        int intersectionCount = 0;

        if (discriminant >= 0) {
            firstIntersectionX = ((determinant * dY) + (sign * dX * Math.sqrt(discriminant))) / (segmentLength * segmentLength);
            secondIntersectionX = ((determinant * dY) - (sign * dX * Math.sqrt(discriminant))) / (segmentLength * segmentLength);

            double yOffset = Math.abs(dY) * Math.sqrt(discriminant);

            firstIntersectionY = ((-determinant * dX) + yOffset) / (segmentLength * segmentLength);
            secondIntersectionY = ((-determinant * dX) - yOffset) / (segmentLength * segmentLength);

            if (firstIntersectionX<Math.min(lineStartX, lineEndX)||firstIntersectionX>Math.max(lineStartX, lineEndX)||firstIntersectionY<Math.min(lineStartY, lineEndY)||firstIntersectionY>Math.max(lineStartY, lineEndY)) {
                firstIntersectionX = null;
                firstIntersectionY = null;
            }
            if (secondIntersectionX<Math.min(lineStartX, lineEndX)||secondIntersectionX>Math.max(lineStartX, lineEndX)||secondIntersectionY<Math.min(lineStartY, lineEndY)||secondIntersectionY>Math.max(lineStartY, lineEndY)) {
                secondIntersectionX = null;
                secondIntersectionY = null;
            }

            if (firstIntersectionX != null) {
                firstIntersectionX += centerX;
                firstIntersectionY += centerY;
                assertEquals(30.83, (double) (Math.round(firstIntersectionY * 100)) / 100);
                assertEquals(18.27, (double) (Math.round(firstIntersectionX * 100)) / 100);
                intersectionCount++;
            }
            if ((discriminant>discriminantTolerance || firstIntersectionX == null) && secondIntersectionX != null) {
                secondIntersectionX += centerX;
                secondIntersectionY += centerY;
                assertEquals(22.60, (double) (Math.round(secondIntersectionY * 100)) / 100);
                assertEquals(9.58, (double) (Math.round(secondIntersectionX * 100)) / 100);
                intersectionCount++;
            }
            assertEquals(2, intersectionCount);
        }
    }
}
