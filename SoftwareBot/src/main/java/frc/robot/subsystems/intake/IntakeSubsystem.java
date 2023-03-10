// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final double MAX_VOLTS = 12;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake/inputs", inputs);
    }

    public void setPower(double power) {
        double volts = power * MAX_VOLTS;
        setVoltage(volts);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public double getFeedRate() {
        return inputs.intakeFeedRadPerSec;
    }
}
