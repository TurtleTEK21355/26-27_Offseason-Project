package org.firstinspires.ftc.teamcode.library.actuator.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * <h1>LimitedMotor</h1>
 * <h3>Motor has mechanical limits, but has no limit switches</h3>
 * <hr/>
 * <p>Uses a bottom limit, top limit, and tolerance to handle the movement of a {@link DcMotorEx}.
 * This specific type is for motors with mechanical movement limits but no limit switches.
 * To create a limit switched motor, see {@link LimitSwitchMotor}.
 * This method allows for absolute and relative movement.
 * You can also set position to a percentage of the mechanical range.</p>
 * <p>0 must be within the range of the motor, although the top limit cannot be 0</p>
 * @since 03/15/2026
 * @version 0.1.0 (03/15/2026)
 * @author ByteOfToast
 */
public class LimitedMotor {
    public final int bottomLimit;
    public final int topLimit;
    private DcMotorEx motor;

    /**
     * Constructs a new limited motor
     * @param motor the passed in motor from robot initialization
     * @param bottomLimit the lower limit of mechanical motion; must be less than or equal to 0
     * @param topLimit the upper limit of mechanical motion; must be greater than 0
     * @param tolerance the tolerance (in encoder ticks) of the positional movement
     */
    public LimitedMotor(DcMotorEx motor, int bottomLimit, int topLimit, int tolerance) {
        if (bottomLimit > 0)
            throw new IllegalArgumentException("Bottom limit must be less than or equal to 0");
        if (topLimit <= 0)
            throw new IllegalArgumentException("Top limit must be greater than 0");
        this.motor = motor;
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setPower(0);
        this.motor.setTargetPositionTolerance(tolerance);
        this.motor.setTargetPosition(0);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.bottomLimit = bottomLimit;
        this.topLimit = topLimit;
    }

    public double getPercentOfLimit(){return (double) getLimitRelativePosition() / (topLimit - bottomLimit);}
    public int getPosition(){return motor.getCurrentPosition();}
    public int getLimitRelativePosition(){return motor.getCurrentPosition()- bottomLimit;}
    public void setPosition(int position, double speed){
        motor.setTargetPosition(Range.clip(position, bottomLimit, topLimit));
        motor.setPower(Math.abs(speed));
    }
    public void setPositionAsPercentOfLimit(double percent, double speed){
        int relativePosition = (int) Math.round(Range.clip(percent, 0, 1)*(topLimit - bottomLimit));
        int absolutePosition = relativePosition+ bottomLimit;
        setPosition(absolutePosition, speed);
    }
    public boolean isBusy(){return motor.isBusy();}
}
