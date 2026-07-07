package org.firstinspires.ftc.teamcode.examples.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.examples.robot.ProgrammingChassis;
import org.firstinspires.ftc.teamcode.library.internal.Pose2D;
import org.firstinspires.ftc.teamcode.library.internal.math.Coordinate2D;
import org.firstinspires.ftc.teamcode.library.internal.math.PathFollower;
import org.firstinspires.ftc.teamcode.library.internal.pid.PController;

import java.util.ArrayList;

@Autonomous(name = "TestCustomPathFollowing")
public class TestCustomPathFollowing extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        ProgrammingChassis robot = ProgrammingChassis.build(hardwareMap);
        robot.getOdometry().resetPosition();
        PController xController = new PController(0.05, 0, 2);
        PController yController = new PController(0.05, 0, 2);
        ArrayList<Coordinate2D> path = new ArrayList<>();
        path.add(new Coordinate2D(0,0));
        path.add(new Coordinate2D(10,0));
        path.add(new Coordinate2D(10,10));
        path.add(new Coordinate2D(0,10));
        path.add(new Coordinate2D(0,0));
        PathFollower pathFollower = new PathFollower(path, xController, yController);
        while (opModeIsActive()) {
            Pose2D pose = robot.getOdometry().getPosition();
            robot.getDrivetrain().control(pathFollower.getYSpeed(pose.y), pathFollower.getXSpeed(pose.x), 0);
        }
    }
}