package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class AlignCardinalDirectionCommand extends CommandBase {

  private static final double POSITION_TOLERANCE_RADIANS = Units.degreesToRadians(2.0);

  private final DrivetrainSubsystem drivetrain;
  private PIDController pid = new PIDController(5, 0, 0);

  public AlignCardinalDirectionCommand(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    double currentAngleRad = drivetrain.getPose().getRotation().getRadians();
    double targetAngleRad;

    if (currentAngleRad < Math.toRadians(45.0) && currentAngleRad >= Math.toRadians(-45.0)) {
      targetAngleRad = 0;
    } else if ( currentAngleRad < Math.toRadians(135.0) && currentAngleRad >= Math.toRadians(45.0)) {
      targetAngleRad = Math.PI / 2; 
    } else if ( currentAngleRad < Math.toRadians(-45.0) && currentAngleRad >= Math.toRadians(-135.0)) {
      targetAngleRad = -Math.PI / 2;
    } else {
      targetAngleRad = Math.PI;
    }
    
    pid.enableContinuousInput(-Math.PI, Math.PI);
    pid.setTolerance(POSITION_TOLERANCE_RADIANS);
    pid.setSetpoint(targetAngleRad);
  }

  @Override
  public void execute() {
    Rotation2d currentRotation = drivetrain.getPose().getRotation();
    double rotationSpeed = pid.calculate(currentRotation.getRadians());
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotationSpeed, currentRotation));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
