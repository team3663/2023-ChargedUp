package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.*;


public class AdjustArmPoseCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final double deltaXMeters;
    private final double deltaYMeters;
    private final double deltaZRad;

    


    public AdjustArmPoseCommand(ArmSubsystem arm, double deltaXMeters, double deltaYMeters, double deltaZRad) {

        this.arm = arm;

        this.deltaXMeters = deltaXMeters;
        this.deltaYMeters = deltaYMeters;
        this.deltaZRad = deltaZRad;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = arm.getTargetPose();        
        Pose2d newPose = new Pose2d(currentPose.getX() + deltaXMeters, currentPose.getY() + deltaYMeters, new Rotation2d(currentPose.getRotation().getRadians() + deltaZRad));

        if (arm.isValidPose(newPose)) {
            arm.setTargetPose(newPose);
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return arm.atTargetPose();
    }

     @Override
    public void end(boolean interrupted) {
        System.out.println("AdjustArmCommand ended; interrupted = " + interrupted);
    }   
}
