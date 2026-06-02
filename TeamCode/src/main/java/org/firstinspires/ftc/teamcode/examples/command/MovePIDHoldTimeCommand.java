package org.firstinspires.ftc.teamcode.examples.command;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.actuator.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.internal.Pose2D;
import org.firstinspires.ftc.teamcode.library.sensor.localization.OTOSSensor;

public class MovePIDHoldTimeCommand extends MovePIDCommand{
    private ElapsedTime holdTimer = new ElapsedTime();
    private final int holdTime;
    private boolean holdTimerStartLock = false;
    public String dataKey = "MovePIDHoldTimeCommand";



    public MovePIDHoldTimeCommand(Pose2D target, int holdTime, double speed, MecanumDrive drivetrain, OTOSSensor otosSensor) {
        super(target, speed, drivetrain, otosSensor);
        this.holdTime = holdTime;
    }

    @Override
    public void loop() {
        super.loop();
        if (super.isCompleted() && !holdTimerStartLock) {
            holdTimer.reset();
            holdTimerStartLock = true;
        }


    }

    @Override
    public boolean isCompleted() {
        return super.isCompleted() && holdTimerStartLock && holdTimer.milliseconds() >= holdTime;
    }

}