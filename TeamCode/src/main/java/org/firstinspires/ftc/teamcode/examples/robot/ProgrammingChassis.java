package org.firstinspires.ftc.teamcode.examples.robot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.library.actuator.drivetrain.MechanumDrive;
import org.firstinspires.ftc.teamcode.library.sensor.localization.OTOSSensor;
import org.firstinspires.ftc.teamcode.library.sensor.localization.Pinpoint;

public class ProgrammingChassis {
    private MechanumDrive drivetrain;
    private OTOSSensor odometry;

    public ProgrammingChassis(MechanumDrive drivetrain, OTOSSensor odometry) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
    }
    public MechanumDrive getDrivetrain(){
        return drivetrain;
    }

    public OTOSSensor getOdometry() {
        return odometry;
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
                new MechanumDrive(
                        hardwareMap.get(DcMotor.class, ProgrammingChassisHardwareName.FRONT_LEFT_MOTOR.getName()),
                        hardwareMap.get(DcMotor.class, ProgrammingChassisHardwareName.FRONT_RIGHT_MOTOR.getName()),
                        hardwareMap.get(DcMotor.class, ProgrammingChassisHardwareName.BACK_LEFT_MOTOR.getName()),
                        hardwareMap.get(DcMotor.class, ProgrammingChassisHardwareName.BACK_RIGHT_MOTOR.getName())
                ),
                new OTOSSensor(hardwareMap.get(SparkFunOTOS.class, ProgrammingChassisHardwareName.ODOMETRY_SENSOR.getName()))
        );
    }
}
