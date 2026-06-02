package org.firstinspires.ftc.teamcode.examples.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.library.actuator.drivetrain.MechanumDrive;
import org.firstinspires.ftc.teamcode.library.actuator.mechanisms.LimitedMotor;
import org.firstinspires.ftc.teamcode.library.sensor.localization.Pinpoint;

public class FirstPollenBot {
    private MechanumDrive drivetrain;
//    private Pinpoint pinpoint;
    private DcMotor viperSlide;
    private Servo gate;
    private DcMotor intake;


    public FirstPollenBot(MechanumDrive drivetrain, DcMotor viperSlide, Servo gate, DcMotor intake) {
        this.drivetrain = drivetrain;
//        this.pinpoint = pinpoint;
        this.viperSlide = viperSlide;
        this.viperSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        this.viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.gate = gate;
        this.gate.setDirection(Servo.Direction.REVERSE);
        this.intake = intake;
        this.intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public MechanumDrive getDrivetrain(){
        return drivetrain;
    }
//    public Pinpoint getPinpoint() {return pinpoint;}
    public DcMotor getIntake(){return intake;}
    public DcMotor getViperSlide(){return viperSlide;}
    public Servo getGate(){return gate;}

    /**
     * this is for building the robot without having to copypaste this around everywhere
     * use like:
     * robot = StateRobot.build() in init
     * if new parts are added then change this
     *
     * @return the robot
     */
    public static FirstPollenBot build(HardwareMap hardwareMap) {
        return new FirstPollenBot(
                new MechanumDrive(
                        hardwareMap.get(DcMotor.class, FirstPollenBotHardwareName.FRONT_LEFT_MOTOR.getName()),
                        hardwareMap.get(DcMotor.class, FirstPollenBotHardwareName.FRONT_RIGHT_MOTOR.getName()),
                        hardwareMap.get(DcMotor.class, FirstPollenBotHardwareName.BACK_LEFT_MOTOR.getName()),
                        hardwareMap.get(DcMotor.class, FirstPollenBotHardwareName.BACK_RIGHT_MOTOR.getName())
                ),
//                new Pinpoint(hardwareMap.get(GoBildaPinpoint.class, FirstPollenBotHardwareName.PINPOINT.getName())
//                ),
                hardwareMap.get(DcMotor.class, FirstPollenBotHardwareName.VIPER_SLIDE.getName()),
                hardwareMap.get(Servo.class, FirstPollenBotHardwareName.GATE_SERVO.getName()),
                hardwareMap.get(DcMotor.class, FirstPollenBotHardwareName.INTAKE.getName())

        );
    }
}
