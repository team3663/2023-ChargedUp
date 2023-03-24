package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utility.GameMode;
import frc.robot.utility.GameMode.ScoringPosition;

public class SetScoringPositionCommand extends CommandBase {

  private final ScoringPosition position;

  
  public SetScoringPositionCommand(ScoringPosition position) {
    this.position = position;
  }

  @Override
  public void initialize() {
    GameMode.setScoringPosition(position);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
