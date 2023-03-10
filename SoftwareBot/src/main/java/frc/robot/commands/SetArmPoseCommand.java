package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;


public class SetArmPoseCommand extends CommandBase {

    private final ArmSubsystem arm;
    private ArmPoseID poseID;
    private Pose2d targetPose;

    public SetArmPoseCommand(ArmSubsystem arm, ArmPoseID poseID) {

        this.arm = arm;
        this.poseID = poseID;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        targetPose = ArmPoseLibrary.get(poseID);
        arm.setTargetPose(targetPose);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return arm.atTargetPose();
    }

     @Override
    public void end(boolean interrupted) {
        System.out.println("SetArmPoseCommand ended; interrupted = " + interrupted);
    }   
}
