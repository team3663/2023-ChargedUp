package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmPoseLibrary;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;
import frc.robot.utility.GameMode;

public class PlaceCommand extends CommandBase {
    
    private final ArmSubsystem arm;

    public PlaceCommand(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        switch (GameMode.getScoringPosition()) {
            case LOW:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.SCORE_LOW));
                break;
            case MIDDLE:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.SCORE_MED));
                break;
            case HIGH:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.SCORE_HI));
                break;
            default:
                System.out.println("Error: Invalid scoring position");
        }
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.STOWED));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
