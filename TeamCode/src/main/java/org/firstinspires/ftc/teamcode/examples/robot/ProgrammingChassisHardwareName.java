package org.firstinspires.ftc.teamcode.examples.robot;

public enum ProgrammingChassisHardwareName {
    FRONT_LEFT_MOTOR("lf"),
    FRONT_RIGHT_MOTOR("rf"),
    BACK_LEFT_MOTOR("lb"),
    BACK_RIGHT_MOTOR("rb"),
    ODOMETRY_SENSOR("otos"),
    LIMELIGHT("limelight"),
    WEBCAM("webcam"),
    PINPOINT("pinpoint"),

    IMU("imu");

    private final String name;

    ProgrammingChassisHardwareName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

}
