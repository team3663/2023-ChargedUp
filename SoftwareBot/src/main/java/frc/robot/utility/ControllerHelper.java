package frc.robot.utility;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;

public class ControllerHelper {

    private static double deadbandWidth = 0.1;
    private static double slowmodeValue = 2;
    
    /**
     * @param value - Raw value read from controller axis to be modified
     * @return clipped and scaled axis value 0to use
     */
    public static double modifyAxis(double value, Supplier<Boolean> slowmode) {
        value = MathUtil.applyDeadband(value, deadbandWidth);
        if (slowmode.get()) {
            return value /= slowmodeValue;
        }
        return Math.copySign(value * value * value, value);
    }
}
