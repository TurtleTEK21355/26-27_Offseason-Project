package org.firstinspires.ftc.teamcode.examples.robot;


public enum FirstPollenBotHardwareName {
    FRONT_LEFT_MOTOR("lf"),
    FRONT_RIGHT_MOTOR("rf"),
    BACK_LEFT_MOTOR("lb"),
    BACK_RIGHT_MOTOR("rb"),
    VIPER_SLIDE("viper"),
    GATE_SERVO("gate"),
    INTAKE("intake"),
    ODOMETRY_SENSOR("otos"),
    LIMELIGHT("limelight"),
    PINPOINT("pinpoint"),
    IMU("imu");

    private final String name;

    FirstPollenBotHardwareName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

}
