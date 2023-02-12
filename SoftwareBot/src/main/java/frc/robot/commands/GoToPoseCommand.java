// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GoToPoseCommand extends CommandBase {

  private PIDController translationXController = new PIDController(4.0, 0.0, 0.0);
  private PIDController translationYController = new PIDController(4.0, 0.0, 0.0);
  private PIDController rotationController = new PIDController(0, 0, 0);

  private Pose2d currentPose;

  private double currentX;
  private double currentY;
  private double currentAngle;

  private double translationXSpeed;
  private double translationYSpeed;
  private double rotationSpeed;

  private double targetX;
  private double targetY;

  private final DrivetrainSubsystem drivetrain;
  /** Creates a new GoToPoseCommand. */
  public GoToPoseCommand(DrivetrainSubsystem drivetrain, Translation2d targetTranslation, Rotation2d targetRotation) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);

    targetX = targetTranslation.getX();
    targetY = targetTranslation.getY();

    translationXController.setSetpoint(targetX); 
    translationYController.setSetpoint(targetY);

    rotationController.setSetpoint(targetRotation.getRadians());
    rotationController.enableContinuousInput(0, 2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = drivetrain.getPose();

    currentX = currentPose.getX();
    currentY = currentPose.getY();
    currentAngle = currentPose.getRotation().getRadians();

    translationXSpeed = translationXController.calculate(currentX);
    translationYSpeed = translationYController.calculate(currentY);
    rotationSpeed = rotationController.calculate(currentAngle);

    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationXSpeed, translationYSpeed, rotationSpeed, drivetrain.getPose().getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(currentX - targetX) < 0.01);
  }
}
