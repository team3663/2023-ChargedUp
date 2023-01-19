// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDrivetrainCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final DrivetrainSubsystem drivetrainSubsystem;

    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
//    drivetrainSubsystem = new DrivetrainSubsystem(new GyroIOPigeon2(Constants.DRIVETRAIN_GYRO_ID));

        drivetrainSubsystem = new DrivetrainSubsystem(new GyroIO() {
        },
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim());

        drivetrainSubsystem.setDefaultCommand(new DefaultDrivetrainCommand(drivetrainSubsystem,
                () -> modifyJoystickAxis(-driverController.getLeftY()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> modifyJoystickAxis(-driverController.getLeftX()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> modifyJoystickAxis(-driverController.getRightX()) * drivetrainSubsystem.getMaxAngularVelocityRadPerSec()
        ));

        // Configure the trigger bindings
        configureBindings();
    }

    private double modifyJoystickAxis(double value) {
        value = -MathUtil.applyDeadband(value, 0.1);
        return Math.copySign(value * value, value);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
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
