// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class EjectGamePieceCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private long startTime;
    private long ejectDuration;
    private final double EJECT_POWER = -1.0;

    public EjectGamePieceCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);

        ejectDuration = 500;
    }

    public EjectGamePieceCommand(IntakeSubsystem intake, long ejectDuration) {
        this.intake = intake;
        addRequirements(intake);

        this.ejectDuration = ejectDuration;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        intake.setPower(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.setPower(EJECT_POWER); // Positive to intake, negative to eject
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) > ejectDuration;
    }
}
