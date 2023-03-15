package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeGamePieceCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final double INTAKE_POWER = 1.0;

    private long durationMs;
    private long endTimeMs;

    public IntakeGamePieceCommand(IntakeSubsystem intake, long durationMs) {
        this.intake = intake;
        this.durationMs = durationMs;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        endTimeMs = System.currentTimeMillis() + durationMs;
        intake.setPower(INTAKE_POWER);
    }

    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= endTimeMs;
    }
}
