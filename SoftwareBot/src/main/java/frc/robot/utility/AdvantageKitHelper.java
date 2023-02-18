package frc.robot.utility;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.Robot;

public class AdvantageKitHelper {

    /**
     * setupLogger - Configure AdvantageKit logging for the robot.
     * 
    * @param isCompetition - Boolean flag telling whether we are running in competition or practice mode
    */
    public static Logger setupLogger(boolean isCompetition) {
        Logger logger = Logger.getInstance();

        // If this is a physical robot (with a Rio) then we can log to a USB drive.
        if (Robot.isReal()) {
            logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        }

        // We don't add the NT4 receiver in competition matches to reduce network traffic.
        if (!isCompetition) {
            logger.addDataReceiver(new NT4Publisher());
        }

        return logger;
    }
}
