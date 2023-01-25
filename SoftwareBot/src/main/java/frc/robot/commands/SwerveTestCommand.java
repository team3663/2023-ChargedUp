package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class SwerveTestCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    private double theta = 0;
    private double increment = (2 * Math.PI) / 90; // Step in 4 degree increments

    public SwerveTestCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        theta += increment;
        double xVelocity = Math.cos(theta) * drivetrain.getMaxTranslationalVelocityMetersPerSecond();
        double yVelocity = Math.sin(theta) * drivetrain.getMaxTranslationalVelocityMetersPerSecond();

        ChassisSpeeds chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                0.0,
                drivetrain.getPose().getRotation()
        );

        drivetrain.setTargetChassisVelocity(chassisVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setTargetChassisVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false;
      }
    
}
