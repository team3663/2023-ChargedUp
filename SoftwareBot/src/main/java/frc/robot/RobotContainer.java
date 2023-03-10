// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerPorts;
import frc.robot.commands.AdjustArmPoseCommand;
import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.DefaultDrivetrainCommand;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.IntakeFeedCommand;
import frc.robot.commands.SetArmPoseCommand;
import frc.robot.photonvision.IPhotonVision;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.utility.AutoCommandChooser;
import frc.robot.utility.ControllerHelper;
import frc.robot.utility.RobotIdentity;
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
    private final CommandXboxController operatorController = new CommandXboxController(ControllerPorts.OPERATOR);
    private AutoCommandChooser autoChooser;

    // Subsystems       
    private DrivetrainSubsystem drivetrainSubsystem;
    private ArmSubsystem armSubsystem;
    private LedSubsystem ledSubsystem;
    private IntakeSubsystem intakeSubsystem;

    // Utilities
    private IPhotonVision photonvision;

    // Commands

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
        
        RobotIdentity identity = RobotIdentity.getIdentity();

        photonvision = SubsystemFactory.createPhotonvision(identity);


        armSubsystem = config.getArm().createSubsystem();
        intakeSubsystem = config.getIntake().createSubsystem();

        drivetrainSubsystem = config.getDrivetrain().createSubsystem(photonvision);
        drivetrainSubsystem.setPhotonvision(photonvision);
        ledSubsystem =  new LedSubsystem(Constants.DioIds.RED_LED_ID, Constants.DioIds.GREEN_LED_ID, Constants.DioIds.BLUE_LED_ID);
    }

    private void createCommands() {
        // Initialize the auto command builder
        AutoCommandFactory.init(drivetrainSubsystem, armSubsystem, intakeSubsystem);

        // Create the default drive command and attach it to the drivetrain subsystem.
        Supplier<Boolean> isSlowmode = () -> driverController.rightBumper().getAsBoolean();
        drivetrainSubsystem.setDefaultCommand(new DefaultDrivetrainCommand(drivetrainSubsystem,
                () -> ControllerHelper.modifyAxis(-driverController.getLeftY(), isSlowmode) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(-driverController.getLeftX(), isSlowmode) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> ControllerHelper.modifyAxis(-driverController.getRightX(), isSlowmode) * drivetrainSubsystem.getMaxAngularVelocityRadPerSec()
        ));

        // Create the default command for the LED subsystem attach it.
        DefaultLedCommand ledCommand = new DefaultLedCommand(ledSubsystem);
        ledSubsystem.setDefaultCommand(ledCommand);
    }

    private void configureBindings() {

        //
        // Driver controller bindings
        //

        // Button to reset the robot's pose to a default starting point.  Handy when running in the simulator and 
        // you accidently lose the robot outside the game field, should NOT be configured in the competition bot.
        if (Robot.isSimulation()) {
            driverController.start().onTrue(new InstantCommand(() -> drivetrainSubsystem.resetPose(new Pose2d())));
        }

        // This command resets the gyro's yaw value. As it does not interfere with pose estimation it is safe for competition use.
        driverController.back().onTrue(new InstantCommand(
            () -> drivetrainSubsystem.resetPose(new Pose2d(drivetrainSubsystem.getPose().getX(), drivetrainSubsystem.getPose().getY(), new Rotation2d()))
        ));

        // driverController.povUp().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.SCORE_HI));
        driverController.povLeft().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.STOWED));
        // driverController.povRight().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.SCORE_MED));
        driverController.povDown().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.SCORE_FLOOR));
        
        driverController.a().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.SUBSTATION_PICKUP));
        driverController.b().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.FLOOR_PICKUP));
        driverController.x().onTrue(new InstantCommand(() -> GameModeUtil.set(GamePiece.CUBE)));
        driverController.y().onTrue(new InstantCommand(() -> GameModeUtil.set(GamePiece.CONE)));

        driverController.leftTrigger().whileTrue(new IntakeFeedCommand(intakeSubsystem, () -> 0.25));
        driverController.rightTrigger().whileTrue(new IntakeFeedCommand(intakeSubsystem, () -> -0.25));
        
        //
        // Operator controller bindings
        //

        operatorController.y().onTrue(new InstantCommand(() -> armSubsystem.logTargetPose()));
        
        operatorController.povUp().onTrue(new AdjustArmPoseCommand(armSubsystem, 0, 0.025, 0));
        operatorController.povDown().onTrue(new AdjustArmPoseCommand(armSubsystem, 0, -0.025, 0));   
        operatorController.povLeft().onTrue(new AdjustArmPoseCommand(armSubsystem, -0.025, 0, 0));
        operatorController.povRight().onTrue(new AdjustArmPoseCommand(armSubsystem, 0.025, 0, 0));
        operatorController.leftBumper().onTrue(new AdjustArmPoseCommand(armSubsystem, 0, 0, Units.degreesToRadians(2)));
        operatorController.rightBumper().onTrue(new AdjustArmPoseCommand(armSubsystem, 0, 0, Units.degreesToRadians(-2)));
    }

    private void setupAutoChooser() {
        autoChooser = new AutoCommandChooser();

        // Register all the supported auto commands
        autoChooser.registerDefaultCreator("Do Nothing", () -> AutoCommandFactory.createNullAuto());
        autoChooser.registerCreator("Place Only", () -> AutoCommandFactory.createPlaceOnlyAuto());
        autoChooser.registerCreator("Balance", () -> AutoCommandFactory.createBalanceAuto());

        // Test auto commands that we only register with the chooser if we are not running in competition
        if (!Constants.COMPETITION_MODE) {
            autoChooser.registerCreator("Test", () -> AutoCommandFactory.createTestAuto());
        }

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
