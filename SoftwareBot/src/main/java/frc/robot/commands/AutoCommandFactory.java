
package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public final class AutoCommandFactory {
    private static final PIDConstants AUTO_TRANSLATION_PID_CONSTANTS = new PIDConstants(2.5, 0.0, 0.0);
    private static final PIDConstants AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(2.0, 0.0, 0.0);

    private static PathConstraints pathConstraints = new PathConstraints(1.0, 1.0);

    private static HashMap<String, Command> eventMap = new HashMap<>();
    private static SwerveAutoBuilder builder;

    public static void init(DrivetrainSubsystem drivetrain) {

        builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                AUTO_TRANSLATION_PID_CONSTANTS,
                AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                true,
                drivetrain);
    }

    public static Command createNullAuto() {
        return null;
    }
    
    public static Command createCommonsTestAuto() {
        return builder.fullAuto(PathPlanner.loadPath("CommonsTestPath", pathConstraints));
    }

    public static Command createCommonsRotationTestAuto() {
        return builder.fullAuto(PathPlanner.loadPath("CommonsRotationTestPath", pathConstraints));
    }

    // Default constructor that just throws an exception if you attempt to create an
    // instace of this class.
    private AutoCommandFactory() {
        throw new UnsupportedOperationException("This is a static class, you cannont instantiate it.");
    }
}
