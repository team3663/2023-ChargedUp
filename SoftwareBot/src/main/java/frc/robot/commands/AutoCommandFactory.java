
package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.utility.GamePiece;

public final class AutoCommandFactory {
    private static final PIDConstants AUTO_TRANSLATION_PID_CONSTANTS = new PIDConstants(2.5, 0.0, 0.0);
    private static final PIDConstants AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(2.0, 0.0, 0.0);

    private static PathConstraints pathConstraints = new PathConstraints(1.0, 1.0);

    private static HashMap<String, Command> eventMap = new HashMap<>();
    private static SwerveAutoBuilder builder;

    private static DrivetrainSubsystem drivetrain;
    private static ArmSubsystem arm;
    private static IntakeSubsystem intake;

    public static void init(DrivetrainSubsystem drivetrain, ArmSubsystem arm, IntakeSubsystem intake) {

        AutoCommandFactory.drivetrain = drivetrain;
        AutoCommandFactory.arm = arm;
        AutoCommandFactory.intake = intake;

        builder = new SwerveAutoBuilder(
                () -> drivetrain.getPose(),
                (pose) -> drivetrain.resetPose(pose),
                AUTO_TRANSLATION_PID_CONSTANTS,
                AUTO_ROTATION_PID_CONSTANTS,
                (chassisSpeeds) -> drivetrain.setTargetChassisVelocity(chassisSpeeds),
                eventMap,
                true,
                drivetrain);
    }

    /**
     * Autonomous command that just sits there and does nothing.
     */
    public static Command createNullAuto() {
        return null;
    }

    /**
     * Autonomous command that just places our preloaded game piece and nothing else.
     */
    public static SequentialCommandGroup createPlaceOnlyAuto() {

        SequentialCommandGroup group = new SequentialCommandGroup();

        // Ensure we are in the game piece mode associated with the preloaded game piece.
        Command cmd = new SetGameModeCommand(GamePiece.CUBE);
        group.addCommands(cmd);
        
        // Position the arm to score the preloaded game piece
        cmd = new SetArmPoseCommand(arm, ArmPoseID.SCORE_HI);
        group.addCommands(cmd);

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
    public static SequentialCommandGroup createBalanceAuto() {

        // We start with the PlaceOnly auto command and add to it.
        SequentialCommandGroup group = createPlaceOnlyAuto();

        // Move until we are far enough on the charging station that the robot is tilted.
        Command cmd = builder.fullAuto(PathPlanner.loadPath("ChargeStationAuto", pathConstraints));
        group.addCommands(cmd);

        // Balance on the charging station
        cmd = new AutoBalanceCommand(drivetrain);
        group.addCommands(cmd);

        return group;
    }

    public static Command createTestAuto() {
        return builder.fullAuto(PathPlanner.loadPath("TestPath", pathConstraints));
    }

    // Default constructor that just throws an exception if you attempt to create an
    // instace of this class.
    private AutoCommandFactory() {
        throw new UnsupportedOperationException("This is a static class, you cannot instantiate it.");
    }
}
