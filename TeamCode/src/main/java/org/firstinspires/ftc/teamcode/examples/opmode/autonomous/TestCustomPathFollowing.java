package org.firstinspires.ftc.teamcode.examples.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.examples.robot.ProgrammingChassis;
import org.firstinspires.ftc.teamcode.library.internal.Pose2D;
import org.firstinspires.ftc.teamcode.library.internal.math.Coordinate2D;
import org.firstinspires.ftc.teamcode.library.internal.math.PathFollower;
import org.firstinspires.ftc.teamcode.library.internal.pid.PController;
import org.firstinspires.ftc.teamcode.library.internal.telemetry.TelemetryPasser;

import java.util.ArrayList;

@Autonomous(name = "TestCustomPathFollowing")
public class TestCustomPathFollowing extends LinearOpMode {

    @Override
    public void runOpMode() {
        TelemetryPasser.telemetry = telemetry;
        waitForStart();
        ProgrammingChassis robot = ProgrammingChassis.build(hardwareMap);
        robot.getPinpoint().resetPosition();
        PController xController = new PController(0.1, 0, 2);
        PController yController = new PController(0.1, 0, 2);
        ArrayList<Coordinate2D> path = new ArrayList<>();
        path.add(new Coordinate2D(0,0));
        path.add(new Coordinate2D(-24,0));
        path.add(new Coordinate2D(-24, 24));
        path.add(new Coordinate2D(0,24));
        path.add(new Coordinate2D(0,0));
        PathFollower pathFollower = new PathFollower(path, xController, yController);
        Pose2D pose;
        while (opModeIsActive()) {
            pose = robot.getPinpoint().getPosition();
            robot.getPinpoint().positionTelemetry();

            pathFollower.updatePControllerTarget(pose);
            pathFollower.targetTelemetry();

            robot.getDrivetrain().simulateFCControl(pathFollower.getYSpeed(pose.y), pathFollower.getXSpeed(pose.x), 0, pose.h);
//            robot.getDrivetrain().fcControl(pathFollower.getYSpeed(pose.y), pathFollower.getXSpeed(pose.x), 0, pose.h+90);
//            robot.getDrivetrain().powerTelemetry();

            telemetry.update();
        }
    }
}