// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax.IdleMode;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double intakeFeedRadPerSec;
        public double intakeCurrentDrawAmps;
        public double intakeAppliedVoltage;
    }

    default void updateInputs(IntakeIOInputs inputs) {
    }
    
    default void setVoltage(double volts) {
    }

    default void setIdleMode(IdleMode mode) {
    }
}
