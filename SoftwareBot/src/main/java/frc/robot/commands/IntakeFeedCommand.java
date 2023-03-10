// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class IntakeFeedCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final Supplier<Double> percentOutput;

    /** Creates a new IntakeFeedCommand. */
    public IntakeFeedCommand(IntakeSubsystem intake, Supplier<Double> percentOutput) {
        this.intake = intake;
        this.percentOutput = percentOutput;

        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.setPower(percentOutput.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
