package org.firstinspires.ftc.teamcode.library.actuator.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.examples.Constants;
import org.firstinspires.ftc.teamcode.library.internal.Pose2D;
import org.firstinspires.ftc.teamcode.library.internal.TelemetryPasser;

/**
 * <h1>Mechanum Drive</h1>
 * <p>This drivetrain type uses 4 mechanum wheels in a standard linear setup.
 * This layout allows for optimal control on a flat surface.
 * Due to the nature of mechanum wheel,
 * this drivetrain type is not optimal for slopes or rough surfaces.</p>
 * <hr/>
 * <h6>Uses:</h4>
 * <ol>
 *     <li>Linear Movement</li>
 *     <li>Strafing</li>
 *     <li>Rotation</li>
 *     <li>Agility</li>
 * </ol>
 * @author ByteOfToast
 * @since 03/15/2026
 * @version 0.0.0 (03/15/2026)
 * @see <a href="https://gm0.org/en/latest/docs/common-mechanisms/drivetrains/holonomic.html">gm zero</a>
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

        control(correctedY, correctedX, h);
    }


    /**
     * Y IS FORWARDS AND BACKWARDS
     * @param y +forwards and -backwards
     * @param x strafe -left and +right
     * @param h turn +right and -left
     */
    public void control(double y, double x, double h) {
        frontRightMotor.setPower(Range.clip(y - x - h, -1, 1));
        frontLeftMotor.setPower(Range.clip(y + x + h, -1, 1));
        backRightMotor.setPower(Range.clip(y + x - h, -1, 1));
        backLeftMotor.setPower(Range.clip(y - x + h, -1, 1));
    }

    /**
     * Y IS FORWARDS AND BACKWARDS
     * @param y +forwards and -backwards
     * @param x strafe -left and +right
     * @param h turn +right and -left
     */
    public void rcControl(double y, double x, double h){
        // Uses dead zone and applies drivetrain exponent index
        x = (Math.abs(x)<0.2) ? 0 : Math.pow(x, Constants.drivetrainExponentIndex);
        y = (Math.abs(y)<0.2) ? 0 : Math.pow(y, Constants.drivetrainExponentIndex);
        h = (Math.abs(h)<0.2) ? 0 : Math.pow(h, Constants.drivetrainExponentIndex);

        // Calculates motor values before being compressed to range
        double fr = y - x - h;
        double fl = y + x + h;
        double br = y + x - h;
        double bl = y - x + h;

        // Calculates scale to compress motor values to range [-1,1]
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        double magnitude = Math.sqrt(x*x + y*y + h*h)/Math.sqrt(3);
        double scale = (max > 0) ? (magnitude / max) : 0;

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