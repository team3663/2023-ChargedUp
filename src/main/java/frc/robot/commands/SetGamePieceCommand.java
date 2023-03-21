package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utility.GameMode;
import frc.robot.utility.GameMode.GamePiece;

public class SetGamePieceCommand extends CommandBase {

  private GamePiece piece;

  public SetGamePieceCommand(GamePiece piece) {
    this.piece = piece;
  }

  @Override
  public void initialize() {
    GameMode.setGamePiece(piece);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
