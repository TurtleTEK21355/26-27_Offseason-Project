package org.firstinspires.ftc.teamcode.examples.command;

import org.firstinspires.ftc.teamcode.examples.robot.Constants;
import org.firstinspires.ftc.teamcode.library.actuator.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.commandsinternal.Command;
import org.firstinspires.ftc.teamcode.library.internal.Pose2D;
import org.firstinspires.ftc.teamcode.library.internal.pid.PIDControllerHeading;
import org.firstinspires.ftc.teamcode.library.internal.pid.PIDControllerSpeedLimit;
import org.firstinspires.ftc.teamcode.library.internal.telemetry.TelemetryString;
import org.firstinspires.ftc.teamcode.library.sensor.localization.OTOSSensor;

public class MovePIDCommand extends Command {
    private Pose2D position;
    double speed;
    MecanumDrive drivetrain;
    OTOSSensor otosSensor;
    PIDControllerSpeedLimit yPID;
    PIDControllerSpeedLimit xPID;
    PIDControllerHeading hPID;

    public String dataKey = "MovePIDCommand";


    /**
     * Regular general-purpose move PID
     * @param target Target position for movement
     * @param speed Maximum PID speed
     * @param drivetrain
     * @param otosSensor
     */
    public MovePIDCommand(Pose2D target, double speed, MecanumDrive drivetrain, OTOSSensor otosSensor) {
        this.drivetrain = drivetrain;
        this.otosSensor = otosSensor;
        this.speed = speed;
        yPID = new PIDControllerSpeedLimit(Constants.getLinearPIDConstants(), target.y, Constants.getPIDTolerance().y, speed);
        xPID = new PIDControllerSpeedLimit(Constants.getLinearPIDConstants(), target.x, Constants.getPIDTolerance().x, speed);
        hPID = new PIDControllerHeading(Constants.getAngularPIDConstants(), target.h, Constants.getPIDTolerance().h, speed);
    }

    @Override
    public void loop() {
        position = otosSensor.getPosition();
        double xCalc = xPID.calculate(position.x);
        double hCalc = hPID.calculate(position.h);
        double yCalc = yPID.calculate(position.y);
        drivetrain.fcControl(yCalc, xCalc, hCalc, position.h);

    }

    @Override
    public String telemetry() {
        TelemetryString string = new TelemetryString();

        string.addData("yPID at Position ", yPID.atTarget(position.y));
        string.addData("xPID at Position ", xPID.atTarget(position.x));
        string.addData("hPID at Position ", hPID.atTarget(position.h));
        string.addData("Robot Position ", position);

        return string.toString();
    }

    @Override
    public boolean isCompleted() {
        return (yPID.atTarget(position.y) && xPID.atTarget(position.x) && hPID.atTarget(position.h));
    }

}