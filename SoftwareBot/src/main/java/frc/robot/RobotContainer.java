// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerPorts;
import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.DefaultDrivetrainCommand;
import frc.robot.commands.DriveCircleCommand;
import frc.robot.commands.SetArmPoseCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.photonvision.IPhotonVision;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.utility.AutoCommandChooser;
import frc.robot.utility.ControllerHelper;
import frc.robot.utility.RobotIdentity;
import frc.robot.utility.config.RobotConfig;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(ControllerPorts.DRIVER);
    private AutoCommandChooser autoChooser;

    // Subsystems       
    private DrivetrainSubsystem drivetrainSubsystem;
    private ArmSubsystem armSubsystem;

    // Utilities
    private IPhotonVision photonvision;

    // Commands
    private DriveCircleCommand driveCircleCommand;
    private DriveToPoseCommand goToPoseCommand;
    private DriveToPoseCommand goToOtherPoseCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotConfig config) {
        createSubsystems(config);
        createCommands();
        configureBindings();
        setupAutoChooser();
    }

    private void createSubsystems(RobotConfig config) {

 //       photonvision = config.getVision().create();
 //       armSubsystem = config.getArm().createSubsystem();


        RobotIdentity identity = RobotIdentity.getIdentity();
        armSubsystem = SubsystemFactory.createArm(identity);
        photonvision = SubsystemFactory.createPhotonvision(identity);

        drivetrainSubsystem = config.getDrivetrain().createSubsystem(photonvision);
        drivetrainSubsystem.setPhotonvision(photonvision);
    }

    private void createCommands() {

        // Setup the factory we use to generate our autonomous commands
        AutoCommandFactory.init(drivetrainSubsystem);

        // Create the default drive command and attach it to the drivetrain subsystem.
        drivetrainSubsystem.setDefaultCommand(new DefaultDrivetrainCommand(drivetrainSubsystem,
                () -> ControllerHelper.modifyAxis(-driverController.getLeftY()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(-driverController.getLeftX()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(-driverController.getRightX()) * drivetrainSubsystem.getMaxAngularVelocityRadPerSec()
        ));

        driveCircleCommand = new DriveCircleCommand(drivetrainSubsystem);

        goToPoseCommand = new DriveToPoseCommand(drivetrainSubsystem, new Translation2d(1, 1), new Rotation2d(0.5));
        goToOtherPoseCommand = new DriveToPoseCommand(drivetrainSubsystem, new Translation2d(0, 0), new Rotation2d(-0.5));
    }

    private void configureBindings() {

        // Button to reset the robot's pose to a default starting point.  Handy when running in the simulator and 
        // you accidently lose the robot outside the game field, should NOT be configured in the competition bot.
        if (Robot.isSimulation()) {
            driverController.start().onTrue(new InstantCommand(() -> drivetrainSubsystem.resetPose(new Pose2d())));
        }

        driverController.back().onTrue(new InstantCommand(
            () -> drivetrainSubsystem.resetPose(new Pose2d(drivetrainSubsystem.getPose().getX(), drivetrainSubsystem.getPose().getY(), new Rotation2d()))
        ));

        driverController.a().whileTrue(driveCircleCommand);

        driverController.povLeft().onTrue(new SetArmPoseCommand(armSubsystem, new Pose2d(0.11, 0.16, Rotation2d.fromDegrees(110.0))));
        driverController.povRight().onTrue(new SetArmPoseCommand(armSubsystem, new Pose2d(0.51, 0.81, Rotation2d.fromDegrees(0.0))));
        // driverController.povUp().onTrue(new SetArmPoseCommand(armSubsystem, new Pose2d(1.5, 1.0, Rotation2d.fromDegrees(45.0))));
        // driverController.povDown().onTrue(new SetArmPoseCommand(armSubsystem, new Pose2d(1.5, 0.2, Rotation2d.fromDegrees(0.0))));
        
        driverController.b().onTrue(goToPoseCommand);
        driverController.y().onTrue(goToOtherPoseCommand);
    }

    private void setupAutoChooser() {
        autoChooser = new AutoCommandChooser();

        // Register all the supported auto commands
        autoChooser.registerDefaultCreator("Do Nothing", () -> AutoCommandFactory.createNullAuto());
        autoChooser.registerCreator("Test Path", () -> AutoCommandFactory.createTestAuto());
        autoChooser.registerCreator("Commons Test Path", () -> AutoCommandFactory.createCommonsTestAuto());
        autoChooser.registerCreator("Commons Rotation Test Path", () -> AutoCommandFactory.createCommonsRotationTestAuto());

        // Setup the chooser in shuffleboard
        autoChooser.setup("Driver", 0, 0, 2, 1);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getAutonomousCommand();
    }
}
