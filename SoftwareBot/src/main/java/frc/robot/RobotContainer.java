// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerPorts;
import frc.robot.commands.DefaultDrivetrainCommand;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.utility.ControllerHelper;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final CommandXboxController driverController =
            new CommandXboxController(ControllerPorts.DRIVER);

    // Subsystems       
    private DrivetrainSubsystem drivetrainSubsystem;

    // Commands

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        createSubsystems();
        createCommands();
        configureBindings();
    }

    private void createSubsystems() {
        if (Robot.isReal()) {
            drivetrainSubsystem = new DrivetrainSubsystem(
                    new GyroIOPigeon2(Constants.CanIds.DRIVETRAIN_PIGEON_ID),
                    new SwerveModuleIOFalcon500(Constants.CanIds.DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR,
                            Constants.CanIds.DRIVETRAIN_FRONT_LEFT_MODULE_STEER_MOTOR,
                            Constants.CanIds.DRIVETRAIN_FRONT_LEFT_MODULE_STEER_ENCODER,
                            Constants.FRONT_LEFT_MODULE_STEER_OFFSET),
                    new SwerveModuleIOFalcon500(Constants.CanIds.DRIVETRAIN_FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                            Constants.CanIds.DRIVETRAIN_FRONT_RIGHT_MODULE_STEER_MOTOR,
                            Constants.CanIds.DRIVETRAIN_FRONT_RIGHT_MODULE_STEER_ENCODER,
                            Constants.FRONT_RIGHT_MODULE_STEER_OFFSET),
                    new SwerveModuleIOFalcon500(Constants.CanIds.DRIVETRAIN_BACK_LEFT_MODULE_DRIVE_MOTOR,
                            Constants.CanIds.DRIVETRAIN_BACK_LEFT_MODULE_STEER_MOTOR,
                            Constants.CanIds.DRIVETRAIN_BACK_LEFT_MODULE_STEER_ENCODER,
                            Constants.BACK_LEFT_MODULE_STEER_OFFSET),
                    new SwerveModuleIOFalcon500(Constants.CanIds.DRIVETRAIN_BACK_RIGHT_MODULE_DRIVE_MOTOR,
                            Constants.CanIds.DRIVETRAIN_BACK_RIGHT_MODULE_STEER_MOTOR,
                            Constants.CanIds.DRIVETRAIN_BACK_RIGHT_MODULE_STEER_ENCODER,
                            Constants.BACK_RIGHT_MODULE_STEER_OFFSET));
        } else {
            drivetrainSubsystem = new DrivetrainSubsystem(new GyroIO() {
            },
                    new SwerveModuleIOSim(),
                    new SwerveModuleIOSim(),
                    new SwerveModuleIOSim(),
                    new SwerveModuleIOSim());
        }

        drivetrainSubsystem.setDefaultCommand(new DefaultDrivetrainCommand(drivetrainSubsystem,
                () -> ControllerHelper.modifyAxis(driverController.getLeftY()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(driverController.getLeftX()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(driverController.getRightX()) * drivetrainSubsystem.getMaxAngularVelocityRadPerSec()
        ));
    }

    private Command createAutoRoutine () {

        PathPlannerTrajectory path = PathPlanner.loadPath("TestPath", new PathConstraints(4, 3));
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            () -> drivetrainSubsystem.getPose(),
            (pose) -> drivetrainSubsystem.resetPose(pose),
            new PIDConstants(5.0, 0, 0),
            new PIDConstants(0.5, 0, 0),
            (chassisSpeeds) -> drivetrainSubsystem.setTargetChassisVelocity(chassisSpeeds),
            eventMap,
            false,
            drivetrainSubsystem
        );

        return autoBuilder.fullAuto(path);
    }

    private void createCommands() {
        
    }

    private void configureBindings() {

        // Button to reset the robot's pose to a default starting point.  Handy when running in the simulator and 
        // you accidently lose the robot outside the game field, should NOT be configured in the competition bot.
        driverController.start().onTrue(new InstantCommand(() -> drivetrainSubsystem.resetPose(new Pose2d())));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return createAutoRoutine();
    }
}
