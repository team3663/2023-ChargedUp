
package frc.robot.commands;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.Constants;

public final class AutoCommandFactory {

    private static PathConstraints pathConstraints = new PathConstraints(1.0, 1.0);
    

    public static Command createNullAuto() {
        return null;
    }
    
    public static Command createTestAuto(DrivetrainSubsystem drivetrain) {

        PathPlannerTrajectory path = PathPlanner.loadPath("TestPath", pathConstraints);
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                Constants.AUTO_TRANSLATION_PID_CONSTANTS,
                Constants.AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                false,
                drivetrain);

        return new InstantCommand(() -> Logger.getInstance().recordOutput("Drivetrain/Trajectory", path)).andThen(builder.fullAuto(path));
    }

    public static Command createRotationTestAuto(DrivetrainSubsystem drivetrain) {

        PathPlannerTrajectory path = PathPlanner.loadPath("RotationTestPath", pathConstraints);
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                Constants.AUTO_TRANSLATION_PID_CONSTANTS,
                Constants.AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                false,
                drivetrain);

                return new InstantCommand(() -> Logger.getInstance().recordOutput("Drivetrain/Trajectory", path)).andThen(builder.fullAuto(path));
    }

    public static Command createDiagonalTestAuto(DrivetrainSubsystem drivetrain) {

        PathPlannerTrajectory path = PathPlanner.loadPath("DiagonalTestPath", pathConstraints);
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                Constants.AUTO_TRANSLATION_PID_CONSTANTS,
                Constants.AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                false,
                drivetrain);

                return new InstantCommand(() -> Logger.getInstance().recordOutput("Drivetrain/Trajectory", path)).andThen(builder.fullAuto(path));
    }

    public static Command createStraightTestAuto(DrivetrainSubsystem drivetrain) {

        PathPlannerTrajectory path = PathPlanner.loadPath("StraightTestPath", pathConstraints);
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                Constants.AUTO_TRANSLATION_PID_CONSTANTS,
                Constants.AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                false,
                drivetrain);

                return new InstantCommand(() -> Logger.getInstance().recordOutput("Drivetrain/Trajectory", path)).andThen(builder.fullAuto(path));
    }

    public static Command createRotationTester(DrivetrainSubsystem drivetrain) {

        PathPlannerTrajectory path = PathPlanner.loadPath("RotationTester", pathConstraints);
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                Constants.AUTO_TRANSLATION_PID_CONSTANTS,
                Constants.AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                false,
                drivetrain);

                return new InstantCommand(() -> Logger.getInstance().recordOutput("Drivetrain/Trajectory", path)).andThen(builder.fullAuto(path));
    }

    // Default constructor that just throws an exception if you attempt to create an
    // instace of this class.
    private AutoCommandFactory() {
        throw new UnsupportedOperationException("This is a static class, you cannont instantiate it.");
    }
}
