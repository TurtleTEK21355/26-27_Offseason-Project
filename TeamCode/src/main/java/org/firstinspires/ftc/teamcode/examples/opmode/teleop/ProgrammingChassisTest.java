package org.firstinspires.ftc.teamcode.examples.opmode.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.examples.robot.ProgrammingChassis;
import org.firstinspires.ftc.teamcode.library.internal.telemetry.TelemetryPasser;

@TeleOp(name = "Programming Chassis Test", group = "test")
public class ProgrammingChassisTest extends LinearOpMode {
    ProgrammingChassis robot;
    private final double XOFFSET = -60;
    private final double YOFFSET = -168;


    @Override
    public void runOpMode() {
        PanelsGamepad virtualGamepad = PanelsGamepad.INSTANCE;
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        TelemetryPasser.telemetry = telemetry;

        robot = ProgrammingChassis.build(hardwareMap);
        robot.getPinpoint().statusTelemetry();
        robot.getPinpoint().setPodOffsets(XOFFSET, YOFFSET);
        robot.getPinpoint().resetPosition();
        robot.getPinpoint().positionTelemetry();
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            Gamepad gamepad1 = virtualGamepad.getFirstManager().asCombinedFTCGamepad(super.gamepad1);
            Gamepad gamepad2 = virtualGamepad.getFirstManager().asCombinedFTCGamepad(super.gamepad2);

            robot.getDrivetrain().rcControl(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            robot.getPinpoint().positionTelemetry();
            telemetry.update();
        }
    }
}