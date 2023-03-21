package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utility.GameMode;
import frc.robot.utility.GameMode.GamePiece;

public class SetGamePieceCommand extends CommandBase {

  private GamePiece piece;

  public SetGamePieceCommand(GamePiece piece) {
    this.piece = piece;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GameMode.setGamePiece(piece);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
