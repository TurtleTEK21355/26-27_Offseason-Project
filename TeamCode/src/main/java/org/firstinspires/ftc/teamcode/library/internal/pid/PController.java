package org.firstinspires.ftc.teamcode.library.internal.pid;

public class PController {

    double kP;
    double target;
    double error;
    double tolerance;
    double previousError = 0;
    double proportional;


    public PController(double kP, double target, double tolerance){
        this.kP = kP;
        this.target = target;
        this.tolerance = tolerance;
    }

    public double calculate(double current){
        error = target - current;
        proportional = error;
        previousError = error;
        return proportional * kP;

    }

    /**
     * tells if the absolute distance from target is greater than the tolerance
     * @param current the current position
     */
    public boolean atTarget(double current){
        double distance = target - current;
        return (Math.abs(distance) <= tolerance);

    }

}
