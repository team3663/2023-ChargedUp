
package frc.robot.commands;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public final class AutoCommandFactory {
    private static final PIDConstants AUTO_TRANSLATION_PID_CONSTANTS = new PIDConstants(2.5, 0.0, 0.0);
    private static final PIDConstants AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(2.0, 0.0, 0.0);

    private static PathConstraints pathConstraints = new PathConstraints(1.0, 1.0);
    
    private static boolean isRedAlliance = DriverStation.getAlliance() == Alliance.Red;

    public static Command createNullAuto() {
        return null;
    }
    
    public static Command createTestAuto(DrivetrainSubsystem drivetrain) {

        PathPlannerTrajectory path = PathPlanner.loadPath("TestPath", pathConstraints);
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                AUTO_TRANSLATION_PID_CONSTANTS,
                AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                isRedAlliance,
                drivetrain);

        return new InstantCommand(() -> Logger.getInstance().recordOutput("Drivetrain/Trajectory", path)).andThen(builder.fullAuto(path));
    }

    public static Command createRotationTestAuto(DrivetrainSubsystem drivetrain) {

        PathPlannerTrajectory path = PathPlanner.loadPath("RotationTestPath", pathConstraints);
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                AUTO_TRANSLATION_PID_CONSTANTS,
                AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                isRedAlliance,
                drivetrain);

                return new InstantCommand(() -> Logger.getInstance().recordOutput("Drivetrain/Trajectory", path)).andThen(builder.fullAuto(path));
    }

    public static Command createDiagonalTestAuto(DrivetrainSubsystem drivetrain) {

        PathPlannerTrajectory path = PathPlanner.loadPath("DiagonalTestPath", pathConstraints);
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                AUTO_TRANSLATION_PID_CONSTANTS,
                AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                isRedAlliance,
                drivetrain);

                return new InstantCommand(() -> Logger.getInstance().recordOutput("Drivetrain/Trajectory", path)).andThen(builder.fullAuto(path));
    }

    public static Command createStraightTestAuto(DrivetrainSubsystem drivetrain) {

        PathPlannerTrajectory path = PathPlanner.loadPath("StraightTestPath", pathConstraints);
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                AUTO_TRANSLATION_PID_CONSTANTS,
                AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                isRedAlliance,
                drivetrain);

                return new InstantCommand(() -> Logger.getInstance().recordOutput("Drivetrain/Trajectory", path)).andThen(builder.fullAuto(path));
    }

    public static Command createRotationTester(DrivetrainSubsystem drivetrain) {

        PathPlannerTrajectory path = PathPlanner.loadPath("RotationTester", pathConstraints);
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                AUTO_TRANSLATION_PID_CONSTANTS,
                AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                isRedAlliance,
                drivetrain);

                return new InstantCommand(() -> Logger.getInstance().recordOutput("Drivetrain/Trajectory", path)).andThen(builder.fullAuto(path));
    }

    // Default constructor that just throws an exception if you attempt to create an
    // instace of this class.
    private AutoCommandFactory() {
        throw new UnsupportedOperationException("This is a static class, you cannont instantiate it.");
    }
}
