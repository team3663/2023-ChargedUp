package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmPoseLibrary;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.utility.GameMode;

public class PickupCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;

    private boolean sequenced = false;

    public PickupCommand(ArmSubsystem arm, IntakeSubsystem intake) {
        this.arm = arm;
        this.intake = intake;

        addRequirements(arm, intake);
    }

    @Override
    public void initialize() {
        sequenced = false;
        switch (GameMode.getPickupLocation()) {
            case FLOOR:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.FLOOR_PICKUP));
                break;
            case SINGLE_STATION:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.SINGLE_STATION_PICKUP));
                break;
            case DOUBLE_STATION:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.INTERMEDIATE));
                sequenced = true;
                break;
            default:
                System.out.println("Error: Invalid scoring position");
        }

        intake.setPower(1.0);
    }

    @Override
    public void execute() {
        if (sequenced) {
            if (arm.atTargetPose()) {
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.DOUBLE_STATION_PICKUP));
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.STOWED));
        intake.setPower(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
