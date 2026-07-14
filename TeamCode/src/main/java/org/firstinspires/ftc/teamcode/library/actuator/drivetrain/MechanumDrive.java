package org.firstinspires.ftc.teamcode.library.actuator.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.examples.robot.Constants;
import org.firstinspires.ftc.teamcode.library.internal.Pose2D;
import org.firstinspires.ftc.teamcode.library.internal.telemetry.TelemetryPasser;

/**
 * <div style="background-color: #0CA366; color: black; border-bottom: 4px dashed black;">
 * <h1>Mechanum Drive</h1>
 * <p>This drivetrain type uses 4 mechanum wheels in a standard linear setup.
 * This layout allows for optimal control on a flat surface.
 * Due to the nature of mechanum wheel,
 * this drivetrain type is not optimal for slopes or rough surfaces.</p>
 * </div>
 * <div style="background-color: #009AD2; color: black; border-top: 4px dashed black;">
 * <h6>Strengths:</h4>
 * <ul>
 *     <li>Linear Movement</li>
 *     <li>Strafing</li>
 *     <li>Rotation</li>
 *     <li>Agility</li>
 * </ul>
 * <h6>Weaknesses</h6>
 * <ul>
 *     <li>Prone to slipping</li>
 *     <li>Only works on flat surfaces</li>
 * </ul>
 * </div>
 * @author Phillip
 * @author Noah
 * @since 03/15/2026
 * @version 0.1.0 (04/02/2026)
 */
public class MechanumDrive {
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;


    public MechanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight){
        this.frontLeftMotor = frontLeft;
        this.frontRightMotor = frontRight;
        this.backLeftMotor = backLeft;
        this.backRightMotor = backRight;
        this.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setWheelDirection(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);

    }
    public void fcControl(double y, double x, double h, double heading) {
        y = Math.pow(y, Constants.drivetrainExponentIndex);
        x = Math.pow(x, Constants.drivetrainExponentIndex);
        h = Math.pow(h, Constants.drivetrainExponentIndex);

        double r = Math.hypot(y, x);
        double theta = Math.atan2(y, x);

        double correctedTheta = theta - Math.toRadians(heading);

        double correctedY = r * Math.sin(correctedTheta);
        double correctedX = r * Math.cos(correctedTheta);

        rcControl(correctedY, correctedX, h);
    }


    /**
     * Y IS FORWARDS AND BACKWARDS
     * @param y +forwards and -backwards
     * @param x strafe -left and +right
     * @param h turn +right and -left
     */
    public void rcControl(double y, double x, double h) {
        frontRightMotor.setPower(Range.clip(y - x - h, -1, 1));
        frontLeftMotor.setPower(Range.clip(y + x + h, -1, 1));
        backRightMotor.setPower(Range.clip(y + x - h, -1, 1));
        backLeftMotor.setPower(Range.clip(y - x + h, -1, 1));
    }
    public void simulateFCControl(double y, double x, double h, double heading) {
        y = Math.pow(y, Constants.drivetrainExponentIndex);
        x = Math.pow(x, Constants.drivetrainExponentIndex);
        h = Math.pow(h, Constants.drivetrainExponentIndex);

        double r = Math.hypot(y, x);
        double theta = Math.atan2(y, x);

        double correctedTheta = theta - Math.toRadians(heading);

        double correctedY = r * Math.sin(correctedTheta);
        double correctedX = r * Math.cos(correctedTheta);

        simulateRCControl(correctedY, correctedX, h);
    }
    public void simulateRCControl(double y, double x, double h) {
        TelemetryPasser.telemetry.addData("Front Right Motor", Range.clip(y - x - h, -1, 1));
        TelemetryPasser.telemetry.addData("Front Left Motor", Range.clip(y + x + h, -1, 1));
        TelemetryPasser.telemetry.addData("Back Right Motor", Range.clip(y + x - h, -1, 1));
        TelemetryPasser.telemetry.addData("Back Left Motor", Range.clip(y - x + h, -1, 1));
    }

    /**
     * calculates and applies optimal motor speeds given xyh input
     * @param input Pose2D xyh input from joysticks or otherwise
     */
    public void rcControl(Pose2D input){
        /*
        * converts all input to positive values and saves whether they need to be flipped after the drivetrain exponential index is applied.
        * This makes sure that an input always results in an output of the same sign.
        * ex. -0.5 won't return 0.25 when the drivetrain exponential index is 2
        * This also ensures that calcVar can always be calculated, since some exponents only accept positive input
        */
        boolean xFlip = false;
        boolean yFlip = false;
        boolean hFlip = false;
        if (input.x<0) {
            xFlip = true;
        } else if (input.y<0) {
            yFlip = true;
        } else if (input.h<0) {
            hFlip = true;
        }
        // applies drivetrain exponent index and flip boolean
        double calcX = Math.pow(Math.abs(input.x), Constants.drivetrainExponentIndex);
        if (xFlip) calcX *= -1;
        double calcY = Math.pow(Math.abs(input.y), Constants.drivetrainExponentIndex);
        if (yFlip) calcY *= -1;
        double calcH = Math.pow(Math.abs(input.h), Constants.drivetrainExponentIndex);
        if (hFlip) calcH *= -1;

        // Calculates motor values before being compressed to range
        double fr = calcY - calcX - calcH;
        double fl = calcY + calcX + calcH;
        double br = calcY + calcX - calcH;
        double bl = calcY - calcX + calcH;

        // Calculates scale to compress motor values to range [-1,1]
        // Also accounts for required speed to move so that the motors don't burn out
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        double magnitude = Math.sqrt(calcX*calcX + calcY*calcY + calcH*calcH)/Math.sqrt(3);
        double scale = (max > Constants.drivetrainMinimumMoveableSpeed) ? (magnitude / max) : 0;

        // Applies scale to set motor powers
        frontRightMotor.setPower(fr*scale);
        frontLeftMotor.setPower(fl*scale);
        backRightMotor.setPower(br*scale);
        backLeftMotor.setPower(bl*scale);
    }

    /**
     * Sends the power of each drive train motor to telemetry
      */
    public void powerTelemetry() {
        TelemetryPasser.telemetry.addLine()
                .addData("Front Left Drivetrain Power: ", frontLeftMotor.getPower())
                .addData("Front Right Drivetrain Power: ", frontRightMotor.getPower())
                .addData("Back Left Drivetrain Power: ", backLeftMotor.getPower())
                .addData("Back Right Drivetrain Power: ", backRightMotor.getPower());
    }

    public void setWheelDirection(DcMotorSimple.Direction lf, DcMotorSimple.Direction rf, DcMotorSimple.Direction lb, DcMotorSimple.Direction rb) {
        this.frontLeftMotor.setDirection(lf);
        this.frontRightMotor.setDirection(rf);
        this.backLeftMotor.setDirection(lb);
        this.backRightMotor.setDirection(rb);

    }
}