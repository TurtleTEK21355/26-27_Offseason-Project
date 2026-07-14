package org.firstinspires.ftc.teamcode.library.internal.math.mock;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.library.internal.Pose2D;

public class MockDT {
    public Pose2D position = new Pose2D(0, 0, 0);

    public double posUnitsPerVel = 1.0;
    public double radPerVel = 0.01;

    public Pose2D velocity = new Pose2D(0, 0, 0);

    public MockDT() {}

    public MockDT(Pose2D position) {
        this.position = position;
    }

    public MockDT(Pose2D position, double posUnitsPerVel, double radPerVel) {
        this.position = position;
        this.posUnitsPerVel = posUnitsPerVel;
        this.radPerVel = radPerVel;
    }

    public Pose2D getPosition() {
        return position;
    }

    public void control(double x, double y, double h) {
        x = Range.clip(x, -1, 1);
        y = Range.clip(y, -1, 1);
        h = Range.clip(h, -1, 1);

        double r = Math.hypot(y, x);
        double theta = Math.atan2(y, x);

        double correctedTheta = theta + position.h;

        double correctedX = r * Math.cos(correctedTheta);
        double correctedY = r * Math.sin(correctedTheta);

        fcControl(correctedX, correctedY, h);
    }

    public void fcControl(double x, double y, double h) {
        x = Range.clip(x, -1, 1);
        y = Range.clip(y, -1, 1);
        h = Range.clip(h, -1, 1);

        velocity.x = x;
        velocity.y = y;
        velocity.h = h;
    }

    public void update() {
        position.x += velocity.x * posUnitsPerVel;
        position.y += velocity.y * posUnitsPerVel;

        // TODO: Wrap heading position
        position.h += velocity.h * radPerVel;

        position.h = ( ( position.h + 180 ) % 360 ) - 180;

        velocity.x = 0;
        velocity.y = 0;
        velocity.h = 0;
    }
}
