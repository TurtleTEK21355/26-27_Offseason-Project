package org.firstinspires.ftc.teamcode.library.internal.pid;

public class PController {

    double proportionalGain;
    double targetPosition;
    double positionalTolerance;
    double proportional;


    public PController(double proportionalGain, double targetPosition, double positionalTolerance){
        this.proportionalGain = proportionalGain;
        this.targetPosition = targetPosition;
        this.positionalTolerance = positionalTolerance;
    }

    public double calculate(double currentPosition){
        return (targetPosition - currentPosition) * proportionalGain;
    }

    public boolean atTarget(double currentPosition){
        double distance = targetPosition - currentPosition;
        return (Math.abs(distance) <= positionalTolerance);
    }

    public double distance(double currentPosition){
        return targetPosition - currentPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }
}