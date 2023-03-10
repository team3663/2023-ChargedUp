package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utility.GameModeUtil;
import frc.robot.utility.GamePiece;

public class SetGameModeCommand extends CommandBase {

  private GamePiece piece;

  public SetGameModeCommand(GamePiece piece) {
    this.piece = piece;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GameModeUtil.set(piece);
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
