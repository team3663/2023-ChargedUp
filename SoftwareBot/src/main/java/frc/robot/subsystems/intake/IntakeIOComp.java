// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utility.config.IntakeConfig;
import frc.robot.utility.config.NeoConfig;
import lombok.Data;
import lombok.EqualsAndHashCode;

/** Add your docs here. */
public class IntakeIOComp implements IntakeIO {

    private final double INTAKE_CURRENT_LIMIT = 40;

    private final CANSparkMax intakeMotor;

    public IntakeIOComp(int intakeMotorId) {
        this(new CANSparkMax(intakeMotorId, MotorType.kBrushless));
    }

    public IntakeIOComp(CANSparkMax intakeMotor) {
        this.intakeMotor = intakeMotor;

        intakeMotor.setSmartCurrentLimit((int) INTAKE_CURRENT_LIMIT);
        intakeMotor.enableVoltageCompensation(12);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeFeedRadPerSec = intakeMotor.getEncoder().getVelocity();
        inputs.intakeAppliedVoltage = intakeMotor.getAppliedOutput() * 12;
        inputs.intakeCurrentDrawAmps = intakeMotor.getOutputCurrent();
    }

    public void setVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    @Data
    @EqualsAndHashCode(callSuper = true)
    public static class HardwareConfig extends IntakeConfig.HardwareConfig {
        private NeoConfig motor;

        public IntakeIO createIO() {
            return new IntakeIOComp(
                motor.create()
            );
        }
    }
}
