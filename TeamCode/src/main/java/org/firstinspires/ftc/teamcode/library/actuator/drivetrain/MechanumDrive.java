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