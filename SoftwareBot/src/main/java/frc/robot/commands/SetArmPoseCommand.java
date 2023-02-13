package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.arm.*;


public class SetArmPoseCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final Pose2d targetPose;

    public SetArmPoseCommand(ArmSubsystem arm, Pose2d targetPose) {

        this.arm = arm;
        this.targetPose = targetPose;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setPose(targetPose);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {

        return arm.getPose().equals(targetPose);
    }

     @Override
    public void end(boolean interrupted) {
    }   
}
