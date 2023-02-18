package frc.robot.utility;

import edu.wpi.first.math.MathUtil;

public class ControllerHelper {

    private static double deadbandWidth = 0.05;
    
    /**
     * @param value - Raw value read from controller axis to be modified
     * @return clipped and scaled axis value 0to use
     */
    public static double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, deadbandWidth);
        return Math.copySign(value * value * value, value);
    }
}
