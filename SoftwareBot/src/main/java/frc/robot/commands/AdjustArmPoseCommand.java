package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.*;


public class AdjustArmPoseCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final Transform2d transform;
    private Pose2d targetPose;


    public AdjustArmPoseCommand(ArmSubsystem arm, double deltaXMeters, double deltaYMeters, double deltaZRad) {

        this.arm = arm;

        Translation2d translation = new Translation2d(deltaXMeters, deltaYMeters);
        Rotation2d rotation = new Rotation2d(deltaZRad);
        transform = new Transform2d(translation, rotation);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = arm.getTargetPose();
        targetPose = currentPose.transformBy(transform);
        arm.setTargetPose(targetPose);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {

        return arm.getCurrentPose().equals(targetPose);
    }

     @Override
    public void end(boolean interrupted) {
    }   
}
