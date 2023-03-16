package frc.robot.utility;

import edu.wpi.first.math.MathUtil;

public class ControllerHelper {

    private static final double defaultScaleCoefficient = 1;

    private double deadbandWidth = 0.1;
    private double scaleCoefficient = defaultScaleCoefficient;
    
    /**
     * @param value - Raw value read from controller axis to be modified
     * @return clipped and scaled axis value 0to use
     */
    public double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, deadbandWidth) * scaleCoefficient;
 
        return Math.copySign(value * value, value);
    }

    /**
     * Set the scaling value we use to reduce robot response to control inputs.
     * 
     * @param coefficient - Value to multiply joystick output by to scale it as desired (default = 1)
     */
    public void setScalingCoefficient(double coefficient)
    {
        scaleCoefficient = coefficient;
    }

    /**
     * Restore the default default scaling coefficient.
     */
    public void restoreDefaultCoefficient()
    {
        scaleCoefficient = defaultScaleCoefficient;
    }
}
