// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.DefaultDrivetrainCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
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

    private void createSubsystems()
    {
        // drivetrainSubsystem = new DrivetrainSubsystem(new GyroIOPigeon2(CanIds.DRIVETRAIN_GYRO));

        drivetrainSubsystem = new DrivetrainSubsystem(new GyroIO() {
        },
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim());

        drivetrainSubsystem.setDefaultCommand(new DefaultDrivetrainCommand(drivetrainSubsystem,
                () -> ControllerHelper.modifyAxis(-driverController.getLeftY()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(-driverController.getLeftX()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(-driverController.getRightX()) * drivetrainSubsystem.getMaxAngularVelocityRadPerSec()
        ));
    }

    private void createCommands()
    {

    }

    private void configureBindings() {

        // Execute a simple statement when the B button is pressed.
        driverController.b().onTrue(new InstantCommand(() -> System.out.println("B button clicked")));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }
}
