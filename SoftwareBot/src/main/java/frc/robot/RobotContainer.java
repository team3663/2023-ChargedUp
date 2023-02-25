// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerPorts;
import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.DefaultDrivetrainCommand;
import frc.robot.commands.DriveCircleCommand;
import frc.robot.commands.SetArmPoseCommand;
import frc.robot.photonvision.IPhotonVision;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.utility.AutoCommandChooser;
import frc.robot.utility.ControllerHelper;
import frc.robot.utility.config.RobotConfig;
import frc.robot.utility.GameModeUtil;
import frc.robot.utility.GamePiece;

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
        photonvision = config.getVision().create();

        armSubsystem = config.getArm().createSubsystem();
        drivetrainSubsystem = config.getDrivetrain().createSubsystem(photonvision);
    }

    private void createCommands() {

        // Setup the factory we use to generate our autonomous commands
        AutoCommandFactory.init(drivetrainSubsystem);

        // Create the default drive command and attach it to the drivetrain subsystem.
        Supplier<Boolean> isSlowmode = () -> driverController.rightBumper().getAsBoolean();
        drivetrainSubsystem.setDefaultCommand(new DefaultDrivetrainCommand(drivetrainSubsystem,
                () -> ControllerHelper.modifyAxis(-driverController.getLeftY(), isSlowmode) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(-driverController.getLeftX(), isSlowmode) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(-driverController.getRightX(), isSlowmode) * drivetrainSubsystem.getMaxAngularVelocityRadPerSec()
        ));

        driveCircleCommand = new DriveCircleCommand(drivetrainSubsystem);
    }

    private void configureBindings() {

        // Button to reset the robot's pose to a default starting point.  Handy when running in the simulator and 
        // you accidently lose the robot outside the game field, should NOT be configured in the competition bot.
        if (Robot.isSimulation()) {
            driverController.start().onTrue(new InstantCommand(() -> drivetrainSubsystem.resetPose(new Pose2d())));
        }

        // This command resets the gyro's yaw value. As it does not interfere with pose estimation it is safe for competition use.
        driverController.back().onTrue(new InstantCommand(
            () -> drivetrainSubsystem.resetPose(new Pose2d(drivetrainSubsystem.getPose().getX(), drivetrainSubsystem.getPose().getY(), new Rotation2d()))
        ));

        driverController.a().whileTrue(driveCircleCommand);

        driverController.povLeft().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.STOWED));
        driverController.povRight().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.SUBSTATION_PICKUP));
        driverController.povDown().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.SCORE_FLOOR));
        
        driverController.x().onTrue(new InstantCommand(() -> GameModeUtil.set(GamePiece.CUBE)));
        driverController.y().onTrue(new InstantCommand(() -> GameModeUtil.set(GamePiece.CONE)));
    }

    private void setupAutoChooser() {
        autoChooser = new AutoCommandChooser();

        // Register all the supported auto commands
        autoChooser.registerDefaultCreator("Do Nothing", () -> AutoCommandFactory.createNullAuto());
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
