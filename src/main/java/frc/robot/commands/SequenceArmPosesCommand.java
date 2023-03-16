package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmPoseLibrary;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;

public class SequenceArmPosesCommand extends CommandBase {

  private final ArmSubsystem arm;
  private final ArmPoseID[] poses;

  private int index;
  private boolean done;
  
  public SequenceArmPosesCommand(ArmSubsystem arm, ArmPoseID... poses) {
    this.arm = arm;
    this.poses = poses;

    if (poses.length == 0) {
      throw new IllegalArgumentException("Poses array is empty");
    }

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    index = 0;
    done = false;

    Pose2d pose = ArmPoseLibrary.get(poses[index]);
    arm.setTargetPose(pose);
  }

  @Override
  public void execute() {

    if (arm.atTargetPose())
    {
      if (++index < poses.length) {
        Pose2d pose = ArmPoseLibrary.get(poses[index]);
        arm.setTargetPose(pose); 
      } else {
        done = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("SequenceArmPosesCommand ended; interrupted = " + interrupted);
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}
