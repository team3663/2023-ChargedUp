package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utility.ControllerHelper;

public class ScaleJoystickCommand extends CommandBase {

  private final ControllerHelper helper;
  private final double scaleValue;

  public ScaleJoystickCommand(ControllerHelper helper, double scaleValue) {
    this.helper = helper;
    this.scaleValue = scaleValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    helper.setScalingCoefficient(scaleValue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    helper.restoreDefaultCoefficient();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
