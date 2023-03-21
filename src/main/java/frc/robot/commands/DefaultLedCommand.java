package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.utility.GameMode;
import frc.robot.utility.GameMode.GamePiece;

public class DefaultLedCommand extends CommandBase {

    private final Color8Bit cubeColor = new Color8Bit(170, 0, 255);
    private final Color8Bit coneColor = new Color8Bit(255, 150, 0);
    private final Color8Bit blackColor = new Color8Bit(0, 0, 0);

    LedSubsystem subsystem;
    
    public DefaultLedCommand(LedSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        GamePiece current = GameMode.getGamePiece();

        if (current == GamePiece.CONE) {
            subsystem.setColor(coneColor);
        }
        else if (current == GamePiece.CUBE) {
            subsystem.setColor(cubeColor);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setColor(blackColor);
    }    
}
