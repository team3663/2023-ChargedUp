package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utility.GameMode;
import frc.robot.utility.GameMode.GamePiece;
import frc.robot.utility.GameMode.PickupLocation;

public class SetGamePieceCommand extends CommandBase {

  private GamePiece piece;
  private PickupLocation pickup;

  public SetGamePieceCommand(GamePiece piece) {
    this.piece = piece;

    // At the moment we only pickup cubes from the floor and cones from the double station
    // So we can choose the correct pickup location here.
    pickup = piece == GamePiece.CUBE ? PickupLocation.FLOOR : PickupLocation.DOUBLE_STATION;
  }

  @Override
  public void initialize() {
    GameMode.setGamePiece(piece);
    GameMode.setPickupLocation(pickup);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
