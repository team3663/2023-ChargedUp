package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.utility.GameModeUtil;
import frc.robot.utility.GamePiece;

public class DefaultIntakeCommand extends CommandBase {

  private static final double HOLD_POWER = 0.05;

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
    GamePiece currentPiece = GameModeUtil.get();
    intake.setPower(currentPiece == GamePiece.CONE ? HOLD_POWER : 0.0);
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
