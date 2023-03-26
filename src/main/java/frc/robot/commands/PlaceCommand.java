package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmPoseLibrary;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;
import frc.robot.utility.GameMode;
import frc.robot.utility.GameMode.ScoringPosition;

public class PlaceCommand extends CommandBase {
    
    private final ArmSubsystem arm;
    
    private boolean sequenced = false;

    public PlaceCommand(ArmSubsystem arm) {
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        sequenced = false;
        switch (GameMode.getScoringPosition()) {
            case LOW:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.SCORE_LOW));
                break;
            case MIDDLE:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.INTERMEDIATE));
                sequenced = true;
                break;
            case HIGH:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.INTERMEDIATE));
                sequenced = true;
                break;
            default:
                System.out.println("Error: Invalid scoring position");
        }
    }

    @Override
    public void execute() {
        if (sequenced) {
            if (arm.atTargetPose()) {
                if (GameMode.getScoringPosition() == ScoringPosition.HIGH) {
                    arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.SCORE_HI));
                } else {
                    arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.SCORE_MED));
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.STOWED));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
