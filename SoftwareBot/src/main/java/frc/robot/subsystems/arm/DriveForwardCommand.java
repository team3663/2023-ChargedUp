// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class DriveForwardCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final double velocity;
    private final double distance;
    private Pose2d startPose;

    private ChassisSpeeds chassisSpeeds;

    /** Creates a new DriveForwardCommand. */
    public DriveForwardCommand(DrivetrainSubsystem drivetrain, double velocity, double distance) {
        this.drivetrain = drivetrain;
        this.velocity = velocity;
        this.distance = distance;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        chassisSpeeds = new ChassisSpeeds(velocity, 0.0, 0.0);

        drivetrain.drive(chassisSpeeds);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
