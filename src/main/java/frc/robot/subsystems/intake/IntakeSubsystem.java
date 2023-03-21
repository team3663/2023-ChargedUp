// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.GameMode;
import frc.robot.utility.GameMode.GamePiece;

public class IntakeSubsystem extends SubsystemBase {

    private final double MAX_VOLTS = 12;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double directionMultiplier;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        directionMultiplier = GameMode.getGamePiece() == GamePiece.CONE ? 1 : -1;
        
        Logger.getInstance().processInputs("Intake/inputs", inputs);
    }

    /**
     * Set the current power level for the intake motor
     * 
     * @param power [-1,1] Positive values intake pieces, negative values eject them
     */
    public void setPower(double power) {
        double volts = power * MAX_VOLTS * directionMultiplier;
        io.setVoltage(volts);
    }

    public void setIdleMode(IdleMode mode) {
        io.setIdleMode(mode);
    }

    public double getFeedRate() {
        return inputs.intakeFeedRadPerSec;
    }
}
