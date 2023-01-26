package frc.robot.utility;

import edu.wpi.first.math.MathUtil;

public class ControllerHelper {

    
    /**
     * @param value - Raw value read from controller axis to be modified
     * @return clipped and scaled axis value to use
     */
    public static double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, 0.1);
        return Math.copySign(value * value, value);
    }
}
