// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class AutoBalanceCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final PIDController controller = new PIDController(0, 0, 0);
    /** Creates a new AutoBalanceCommand. */
    public AutoBalanceCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.drive(new ChassisSpeeds(controller.calculate(drivetrain.getPitch(), 0), 0, 0));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return drivetrain.getPitch() == 0;
    }
}
