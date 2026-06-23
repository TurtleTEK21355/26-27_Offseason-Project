package org.firstinspires.ftc.teamcode.examples.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.library.actuator.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.sensor.localization.OTOSSensor;
import org.firstinspires.ftc.teamcode.library.sensor.localization.Pinpoint;

public class ProgrammingChassis {
    private MecanumDrive drivetrain;
    private OTOSSensor otosSensor;
    private Pinpoint pinpoint;

    public ProgrammingChassis(MecanumDrive drivetrain, Pinpoint pinpoint) {
        this.drivetrain = drivetrain;
        this.pinpoint = pinpoint;
    }
    public MecanumDrive getDrivetrain(){
        return drivetrain;
    }

    public Pinpoint getPinpoint() {
        return pinpoint;
    }


    /**
     * this is for building the robot without having to copypaste this around everywhere
     * use like:
     * robot = StateRobot.build() in init
     * if new parts are added then change this
     *
     * @return the robot
     */
    public static ProgrammingChassis build(HardwareMap hardwareMap) {
        return new ProgrammingChassis(
                new MecanumDrive(
                        hardwareMap.get(DcMotor.class, ProgrammingChassisHardwareName.FRONT_LEFT_MOTOR.getName()),
                        hardwareMap.get(DcMotor.class, ProgrammingChassisHardwareName.FRONT_RIGHT_MOTOR.getName()),
                        hardwareMap.get(DcMotor.class, ProgrammingChassisHardwareName.BACK_LEFT_MOTOR.getName()),
                        hardwareMap.get(DcMotor.class, ProgrammingChassisHardwareName.BACK_RIGHT_MOTOR.getName())
                ),
                new Pinpoint(hardwareMap.get(GoBildaPinpoint.class, ProgrammingChassisHardwareName.PINPOINT.getName()))
        );
    }
}
