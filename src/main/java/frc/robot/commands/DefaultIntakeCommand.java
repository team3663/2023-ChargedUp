package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.utility.GameMode;
import frc.robot.utility.GameMode.GamePiece;

public class DefaultIntakeCommand extends CommandBase {

  private static final double CONE_HOLD_POWER = 0.05;
  private static final double CUBE_HOLD_POWER = 0.05;

  private IntakeSubsystem intake;

  
  public DefaultIntakeCommand(IntakeSubsystem intake) {
    this.intake = intake; 
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // If we are in cone mode then run the intake in at low power to help keep a cone
    // from falling out of the intake.
    GamePiece currentPiece = GameMode.getGamePiece();
    intake.setPower(currentPiece == GamePiece.CONE ? CONE_HOLD_POWER : CUBE_HOLD_POWER);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
