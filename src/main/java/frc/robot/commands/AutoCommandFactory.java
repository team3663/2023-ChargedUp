
package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.utility.GameMode.GamePiece;

public final class AutoCommandFactory {
    private static final PIDConstants AUTO_TRANSLATION_PID_CONSTANTS = new PIDConstants(2.4, 0.0, 0.0);
    private static final PIDConstants AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(7.0, 0.0, 0.25);

    private static PathConstraints normalConstraints = new PathConstraints(4.0, 3.0);
    // private static PathConstraints highSpeedConstraints = new PathConstraints(4.0, 20.0);
    private static PathConstraints experimentalConstraints = new PathConstraints(4.0, 1.8);
    private static PathConstraints intakeConstraints = new PathConstraints(1.0, 3.0);
    private static PathConstraints chargeStationConstraints = new PathConstraints(2.0, 1.0);

    private static HashMap<String, Command> eventMap = new HashMap<>();
    private static SwerveAutoBuilder builder;

    private static DrivetrainSubsystem drivetrain;
    private static ArmSubsystem arm;
    private static IntakeSubsystem intake;

    private static SequentialCommandGroup noBumpSide2Command;
    private static SequentialCommandGroup noBumpSide3Command;

    public static void init(DrivetrainSubsystem drivetrain, ArmSubsystem arm, IntakeSubsystem intake) {

        AutoCommandFactory.drivetrain = drivetrain;
        AutoCommandFactory.arm = arm;
        AutoCommandFactory.intake = intake;

        eventMap.put("floorPickup", new SetArmPoseCommand(arm, ArmPoseID.FLOOR_PICKUP));
        eventMap.put("stowArm", new SetArmPoseCommand(arm, ArmPoseID.STOWED));
        eventMap.put("scoreMed", new SetArmPoseCommand(arm, ArmPoseID.SCORE_MED));
        eventMap.put("runIntake", new IntakeGamePieceCommand(intake, 5000));
        eventMap.put("eject", new EjectGamePieceCommand(intake));
        eventMap.put("ejectSmol", new EjectGamePieceCommand(intake, -0.5, "Yes"));

        builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                AUTO_TRANSLATION_PID_CONSTANTS,
                AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                true,
                drivetrain);

        // Pre-cache auto commands that take a significant length of time to generate
        noBumpSide3Command = createNoBumpSide3Auto();
        noBumpSide2Command = createNoBumpSide2Auto();
    }

    /**
     * Autonomous command that just sits there and does nothing (except unpark the arm)
     */
    public static SequentialCommandGroup createNullAuto() {
        SequentialCommandGroup group = new SequentialCommandGroup();

        // Raise the arm from its resting position to release the kick-stand
        Command cmd = new SetArmPoseCommand(arm, ArmPoseID.RELEASE);
        group.addCommands(cmd);

        // Raise the arm from its resting position to release the kick-stand
        cmd = new SetArmPoseCommand(arm, ArmPoseID.STOWED);
        group.addCommands(cmd);

        return group;
    }

    /**
     * Autonomous command that just places our preloaded game piece and nothing else.
     */
    public static SequentialCommandGroup createPlaceOnlyAuto() {

        Command cmd;

        SequentialCommandGroup group = new SequentialCommandGroup();

        // Raise the arm from its resting position to release the kick-stand
        // cmd = new SetArmPoseCommand(arm, ArmPoseID.RELEASE);
        // group.addCommands(cmd);

        // Ensure we are in the game piece mode associated with the preloaded game piece (always a cube)
        cmd = new SetGamePieceCommand(GamePiece.CUBE);
        group.addCommands(cmd);

        // Hold onto the cube
        group.addCommands(new InstantCommand(() -> intake.setPower(0.1)));

        // Position the arm to score the preloaded game piece
        cmd = new SequenceArmPosesCommand(arm, ArmPoseID.PLACE_INTERMEDIATE, ArmPoseID.SCORE_HI);
        group.addCommands(cmd);

        // Wait for the arm to stabilize
        // cmd = new WaitCommand(2);
        // group.addCommands(cmd);

        // Eject the preloaded game piece
        cmd = new EjectGamePieceCommand(intake);
        group.addCommands(cmd);

        // Return the arm to the stowed position
        cmd = new SetArmPoseCommand(arm, ArmPoseID.STOWED);
        group.addCommands(cmd);

        return group;
    }

    /**
     * Autonomous command that places our preloaded game piece and then balances on 
     * the charging station.
     */
    public static SequentialCommandGroup createMidBalanceAuto() {

        // We start with the PlaceOnly auto command and add to it.
        SequentialCommandGroup group = createPlaceOnlyAuto();

        // Move until we are far enough on the charging station that the robot is tilted.
        Command cmd = builder.fullAuto(PathPlanner.loadPath("ChargeStation", chargeStationConstraints));
        group.addCommands(cmd);

        // Balance on the charging station
        cmd = new AutoBalanceCommand(drivetrain);
        group.addCommands(cmd);

        return group;
    }

    /**
     * Autonomous command that places our preloaded game piece, drives over 
     * the charging station (out of the community) then balance on the charge station.
     */
    public static SequentialCommandGroup createMidMobilityBalanceAuto() {

        // We start with the PlaceOnly auto command and add to it.
        SequentialCommandGroup group = createPlaceOnlyAuto();

        // Move over the charging station, out of the community, then reverse back on the charge station.
        Command cmd = builder.fullAuto(PathPlanner.loadPath("OverChargeStation", chargeStationConstraints));
        group.addCommands(cmd);

        // Balance on the charging station
        cmd = new AutoBalanceCommand(drivetrain);
        group.addCommands(cmd);

        return group;
    }

    public static SequentialCommandGroup createBumpSideAuto() {

        // We start with the PlaceOnly auto and add to it
        SequentialCommandGroup group = createPlaceOnlyAuto();

        Command cmd = builder.fullAuto(PathPlanner.loadPath("LowSide", normalConstraints));
        group.addCommands(cmd);

        return group;
    }

    public static SequentialCommandGroup createBumpSide2Auto() {

        SequentialCommandGroup group = new SequentialCommandGroup();

        Command cmd;

        // We start with the place-only auto
        group.addCommands(createPlaceOnlyAuto());

        // Go to and pickup the cube
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Bump-2", normalConstraints, intakeConstraints);
        cmd = builder.fullAuto(pathGroup.get(0));
        group.addCommands(cmd);
        cmd = builder.fullAuto(pathGroup.get(1));
        group.addCommands(new ParallelRaceGroup(cmd, new IntakeGamePieceCommand(intake, 10000)));
        group.addCommands(new InstantCommand(() -> intake.setPower(0.1)));

        // Experimental path
        // cmd = builder.fullAuto(PathPlanner.loadPath("Bump-2-Experimental", normalConstraints));

        // Return to community and score
        cmd = builder.fullAuto(PathPlanner.loadPath("Bump-2-Return", normalConstraints));
        group.addCommands(cmd);

        //Score the piece
        cmd = new SetArmPoseCommand(arm, ArmPoseID.SCORE_LOW);
        group.addCommands(cmd);

        cmd = new EjectGamePieceCommand(intake);
        group.addCommands(cmd);

        // Stow the arm
        cmd = new SetArmPoseCommand(arm, ArmPoseID.STOWED);
        group.addCommands(cmd);

        return group;
    }

    public static SequentialCommandGroup createNoBumpSide1Auto() {

        SequentialCommandGroup group = createPlaceOnlyAuto();

        Command cmd = builder.fullAuto(PathPlanner.loadPath("HighSide1", normalConstraints));
        group.addCommands(cmd);

        return group;
    }

    public static SequentialCommandGroup createNoBumpSide2Auto() {

        SequentialCommandGroup group = new SequentialCommandGroup();

        Command cmd;

        // Raise the arm from its resting position to release the kick-stand
        // cmd = new SetArmPoseCommand(arm, ArmPoseID.RELEASE);
        // group.addCommands(cmd);      

        // Ensure we are in the game piece mode associated with the preloaded game piece
        cmd = new SetGamePieceCommand(GamePiece.CONE);
        group.addCommands(cmd);

        // Hold on to the cone
        group.addCommands(new InstantCommand(() -> intake.setPower(0.1)));

        // Position the arm to score the preloaded game piece
        cmd = new SequenceArmPosesCommand(arm, ArmPoseID.PLACE_INTERMEDIATE, ArmPoseID.SCORE_HI);
        group.addCommands(cmd);

        // Wait for the arm to stabilize
        cmd = new WaitCommand(0.5);
        group.addCommands(cmd);

        // Eject the preloaded game piece
        cmd = new EjectGamePieceCommand(intake);
        group.addCommands(cmd);

        // Return the arm to the stowed position
        cmd = new SetArmPoseCommand(arm, ArmPoseID.STOWED);
        group.addCommands(cmd);

        cmd = new SetGamePieceCommand(GamePiece.CUBE);
        group.addCommands(cmd);

        // Go to and pickup the cube
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("HighSide2", normalConstraints, intakeConstraints);
        cmd = builder.fullAuto(pathGroup.get(0));
        group.addCommands(cmd);
        cmd = builder.fullAuto(pathGroup.get(1));
        group.addCommands(new ParallelRaceGroup(cmd, new IntakeGamePieceCommand(intake, 4000)));
        group.addCommands(new InstantCommand(() -> intake.setPower(0.1)));

        // Return to community
        cmd = builder.fullAuto(PathPlanner.loadPath("HighSide2Return", normalConstraints));
        group.addCommands(cmd);

        // Position the arm to score the game piece
        cmd = new SequenceArmPosesCommand(arm, ArmPoseID.PLACE_INTERMEDIATE, ArmPoseID.SCORE_HI);
        group.addCommands(cmd);

        // Wait for the arm to stabilize
        cmd = new WaitCommand(0.25);
        group.addCommands(cmd);

        // Eject the preloaded game piece
        cmd = new EjectGamePieceCommand(intake);
        group.addCommands(cmd);

        // Return the arm to the stowed position
        cmd = new SetArmPoseCommand(arm, ArmPoseID.STOWED);
        group.addCommands(cmd);

        return group;
    }

    public static SequentialCommandGroup getNoBumpSide2Auto() {
        return noBumpSide2Command;
    }

    public static SequentialCommandGroup createNoBumpSide3Auto() {

        SequentialCommandGroup group = new SequentialCommandGroup();

        Command cmd;

        // Ensure we are in the game piece mode associated with the preloaded game piece
        cmd = new SetGamePieceCommand(GamePiece.CONE);
        group.addCommands(cmd);

        // Hold on to the cone
        group.addCommands(new InstantCommand(() -> intake.setPower(0.1)));

        // Position the arm to score the preloaded game piece
        cmd = new SequenceArmPosesCommand(arm, ArmPoseID.PLACE_INTERMEDIATE, ArmPoseID.SCORE_MED);
        group.addCommands(cmd);

        // Wait for the arm to stabilize
        cmd = new WaitCommand(0.15);
        group.addCommands(cmd);

        // Eject the preloaded game piece
        cmd = new EjectGamePieceCommand(intake, 200);
        group.addCommands(cmd);

        cmd = new SetGamePieceCommand(GamePiece.CUBE);
        group.addCommands(cmd);

        cmd = builder.fullAuto(PathPlanner.loadPath("HighSide2Experimental", experimentalConstraints));
        group.addCommands(cmd);
        group.addCommands(new InstantCommand(() -> intake.setPower(0.1)));

        // Return to community
        cmd = builder.fullAuto(PathPlanner.loadPath("HighSide2ReturnAlt", normalConstraints));
        group.addCommands(cmd);

        // Go to and pickup third piece
        cmd = builder.fullAuto(PathPlanner.loadPath("HighSide3", experimentalConstraints));
        group.addCommands(cmd);

        group.addCommands(new InstantCommand(() -> intake.setPower(0.1)));

        // Return to grid
        cmd = builder.fullAuto(PathPlanner.loadPath("HighSide3Return", normalConstraints));
        group.addCommands(cmd);

        // Score the piece
        cmd = new SetArmPoseCommand(arm, ArmPoseID.SCORE_LOW);
        group.addCommands(cmd);

        return group;
    }

    public static SequentialCommandGroup getNoBumpSide3Auto() {
        return noBumpSide3Command;
    }

    public static Command createTestAuto() {
        return builder.fullAuto(PathPlanner.loadPath("TestPath", normalConstraints));
    }

    // Default constructor that just throws an exception if you attempt to create an
    // instace of this class.
    private AutoCommandFactory() {
        throw new UnsupportedOperationException("This is a static class, you cannot instantiate it.");
    }
}
