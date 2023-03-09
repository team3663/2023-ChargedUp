// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.utility.GameModeUtil;
import frc.robot.utility.GamePiece;

public class IntakeFeedCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private double volts;

    /** Creates a new IntakeFeedCommand. */
    public IntakeFeedCommand(IntakeSubsystem intake, double percentOutput) {
        this.intake = intake;
        volts = percentOutput * 12;

        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (GameModeUtil.get() == GamePiece.CONE) {
            volts = -volts;
        }
        intake.setVoltage(volts);
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
