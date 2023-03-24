package frc.robot;

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
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.commands.IntakeFeedCommand;
import frc.robot.commands.ScaleJoystickCommand;
import frc.robot.commands.SequenceArmPosesCommand;
import frc.robot.commands.SetArmPoseCommand;
import frc.robot.commands.SetGamePieceCommand;
import frc.robot.commands.SetScoringPositionCommand;
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
import frc.robot.utility.GameMode.GamePiece;
import frc.robot.utility.GameMode.ScoringPosition;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(ControllerPorts.DRIVER);
    private final CommandXboxController operatorController = new CommandXboxController(ControllerPorts.OPERATOR);
    private final CommandXboxController testController = new CommandXboxController(ControllerPorts.TEST);

    private final ControllerHelper driverHelper = new ControllerHelper();
    
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
        drivetrainSubsystem.setDefaultCommand(new DefaultDrivetrainCommand(drivetrainSubsystem,
                () -> driverHelper.modifyAxis(-driverController.getLeftY()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> driverHelper.modifyAxis(-driverController.getLeftX()) * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
                () -> driverHelper.modifyAxis(-driverController.getRightX()) * drivetrainSubsystem.getMaxAngularVelocityRadPerSec()
        ));

        // Create default command for intake and attach it.
        DefaultIntakeCommand intakeCommand = new DefaultIntakeCommand(intakeSubsystem);
        intakeSubsystem.setDefaultCommand(intakeCommand);

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

        driverController.povLeft().onTrue(new SequenceArmPosesCommand(armSubsystem, ArmPoseID.INTERMEDIATE, ArmPoseID.DOUBLE_STATION_PICKUP));
        driverController.povDown().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.SCORE_LOW));
        driverController.povRight().onTrue(new SequenceArmPosesCommand(armSubsystem, ArmPoseID.INTERMEDIATE, ArmPoseID.SCORE_MED));   
        driverController.povUp().onTrue(new SequenceArmPosesCommand(armSubsystem, ArmPoseID.INTERMEDIATE, ArmPoseID.SCORE_HI));

        driverController.a().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.FLOOR_PICKUP));
        driverController.b().onTrue(new SetArmPoseCommand(armSubsystem, ArmPoseID.STOWED));
        driverController.x().onTrue(new SetGamePieceCommand(GamePiece.CUBE));
        driverController.y().onTrue(new SetGamePieceCommand(GamePiece.CONE));

        driverController.leftTrigger().whileTrue(new IntakeFeedCommand(intakeSubsystem, () -> 0.5));
        driverController.rightTrigger().whileTrue(new IntakeFeedCommand(intakeSubsystem, () -> -1.0));

        // Slow-mode and Slower-mode
        driverController.leftBumper().whileTrue(new ScaleJoystickCommand(driverHelper, 0.75));
        driverController.rightBumper().whileTrue(new ScaleJoystickCommand(driverHelper, 0.5));

        // Snap to cardinal direction on right stick click
        //driverController.rightStick().onTrue(new AlignCardinalDirectionCommand(drivetrainSubsystem));
        
        //
        // Operator controller bindings
        //

        // Set the current game piece we are handling
        operatorController.x().onTrue(new SetGamePieceCommand(GamePiece.CUBE));
        operatorController.y().onTrue(new SetGamePieceCommand(GamePiece.CONE));

        // Set the target scoring position
        operatorController.povUp().onTrue(new SetScoringPositionCommand(ScoringPosition.HIGH));
        operatorController.povRight().onTrue(new SetScoringPositionCommand(ScoringPosition.MIDDLE));       
        operatorController.povDown().onTrue(new SetScoringPositionCommand(ScoringPosition.LOW));   
       
        //
        // Test controller bindings
        //

        if (testController.getHID().isConnected())
        {
            testController.a().onTrue(new InstantCommand(() -> armSubsystem.logPose()));
            
            testController.povUp().onTrue(new AdjustArmPoseCommand(armSubsystem, 0, 0.025, 0));
            testController.povDown().onTrue(new AdjustArmPoseCommand(armSubsystem, 0, -0.025, 0));   
            testController.povLeft().onTrue(new AdjustArmPoseCommand(armSubsystem, -0.025, 0, 0));
            testController.povRight().onTrue(new AdjustArmPoseCommand(armSubsystem, 0.025, 0, 0));
            testController.leftBumper().onTrue(new AdjustArmPoseCommand(armSubsystem, 0, 0, Units.degreesToRadians(2)));
            testController.rightBumper().onTrue(new AdjustArmPoseCommand(armSubsystem, 0, 0, Units.degreesToRadians(-2)));
        }
    }

    private void setupAutoChooser() {
        autoChooser = new AutoCommandChooser();

        // Register all the supported auto commands
        autoChooser.registerDefaultCreator("Do Nothing", () -> AutoCommandFactory.createNullAuto());
        autoChooser.registerCreator("Place Only", () -> AutoCommandFactory.createPlaceOnlyAuto());
        autoChooser.registerCreator("Mid Place-Balance", () -> AutoCommandFactory.createMidBalanceAuto());
        autoChooser.registerCreator("BumpSide Place-Move", () -> AutoCommandFactory.createBumpSideAuto());
        autoChooser.registerCreator("NoBumpSide Place-Move", () -> AutoCommandFactory.createNoBumpSide1Auto());
        //autoChooser.registerCreator("NoBumpSide Place-Move-Place", () -> AutoCommandFactory.createNoBumpSide2Auto());

        // Test auto commands that we only register with the chooser if we are not running in competition
        if (!Constants.COMPETITION_MODE) {
            autoChooser.registerCreator("Test", () -> AutoCommandFactory.createTestAuto());
        }

        // Setup the chooser in shuffleboard
        autoChooser.setup("Driver", 0, 0, 3, 1);
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
