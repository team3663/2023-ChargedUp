// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerPorts;
import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.DefaultDrivetrainCommand;
import frc.robot.commands.DriveCircleCommand;
import frc.robot.commands.GoToPoseCommand;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.utility.AutoCommandChooser;
import frc.robot.utility.ControllerHelper;
import frc.robot.utility.PhotonVisionUtil;
import frc.robot.utility.RobotIdentity;
import org.photonvision.PhotonCamera;

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
    private PhotonVisionUtil photonvision;

    // Commands
    private DriveCircleCommand driveCircleCommand;
    private GoToPoseCommand goToPoseCommand;
    private GoToPoseCommand goToOtherPoseCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        createSubsystems();
        createCommands();
        configureBindings();
        setupAutoChooser();
    }

    private void createSubsystems() {
        RobotIdentity identity = RobotIdentity.getIdentity();

        photonvision = new PhotonVisionUtil(
            new PhotonCamera[] {
                new PhotonCamera("Left_Camera"),
                new PhotonCamera("Right_Camera")
            },
            new Transform3d[] {
                new Transform3d(new Pose3d(), Constants.CameraPoses.LEFT_CAMERA_POSE),
                new Transform3d(new Pose3d(), Constants.CameraPoses.RIGHT_CAMERA_POSE)
            }
        );

        armSubsystem = SubsystemFactory.createArm(identity);
        drivetrainSubsystem = SubsystemFactory.createDrivetrain(identity, photonvision);
    }

    private void createCommands() {

        // Create the default drivce command and attach it to the drivetrain subsystem.
        drivetrainSubsystem.setDefaultCommand(new DefaultDrivetrainCommand(drivetrainSubsystem,
                () -> ControllerHelper.modifyAxis(-driverController.getLeftY()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(-driverController.getLeftX()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(-driverController.getRightX()) * drivetrainSubsystem.getMaxAngularVelocityRadPerSec()
        ));

        driveCircleCommand = new DriveCircleCommand(drivetrainSubsystem);

        goToPoseCommand = new GoToPoseCommand(drivetrainSubsystem, new Translation2d(1, 1), new Rotation2d(0.5));
        goToOtherPoseCommand = new GoToPoseCommand(drivetrainSubsystem, new Translation2d(0, 0), new Rotation2d(-0.5));
    }

    private void configureBindings() {

        // Button to reset the robot's pose to a default starting point.  Handy when running in the simulator and 
        // you accidently lose the robot outside the game field, should NOT be configured in the competition bot.
        driverController.start().onTrue(new InstantCommand(() -> drivetrainSubsystem.resetPose(new Pose2d())));

        driverController.a().whileTrue(driveCircleCommand);
        driverController.b().onTrue(goToPoseCommand);
        driverController.y().onTrue(goToOtherPoseCommand);
    }

    private void setupAutoChooser() {
        autoChooser = new AutoCommandChooser();

        // Register all the supported auto commands
        autoChooser.registerDefaultCreator("Do Nothing", () -> AutoCommandFactory.createNullAuto());
        autoChooser.registerCreator("Test Path", () -> AutoCommandFactory.createTestAuto(drivetrainSubsystem));

        // Setup the chooser in shuffleboard
        autoChooser.setup("Driver", 0, 0, 2, 1);
    }

    public PhotonVisionUtil gePhotonVision () {
        return photonvision;
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
