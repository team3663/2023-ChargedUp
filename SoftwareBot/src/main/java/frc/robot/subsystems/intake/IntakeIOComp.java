// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utility.config.CanCoderConfig;
import frc.robot.utility.config.IntakeConfig;
import frc.robot.utility.config.PLGConfig;
import lombok.Data;
import lombok.EqualsAndHashCode;

/** Add your docs here. */
public class IntakeIOComp implements IntakeIO {

    // Converts RPM into rad/sec
    // TODO: Verify that this works
    private final double ENCODER_VELOCITY_COEFFICIENT = 2 * Math.PI / 60;

    private final double INTAKE_CURRENT_LIMIT = 40;

    private final CANSparkMax intakeMotor;

    private final RelativeEncoder intakeEncoder;

    public IntakeIOComp(int intakeMotorId, int intakeEncoderId) {
        this(new CANSparkMax(intakeMotorId, MotorType.kBrushed), "rio");
    }

    public IntakeIOComp(CANSparkMax intakeMotor, String canBusName) {
        this(intakeMotor, intakeMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 45), "rio");
    }

    public IntakeIOComp(CANSparkMax motor, RelativeEncoder encoder, String CanBusName) {
        intakeMotor = motor;
        intakeEncoder = encoder;

        motor.setSmartCurrentLimit((int) INTAKE_CURRENT_LIMIT);
        motor.enableVoltageCompensation(12);
        motor.setInverted(false);

        encoder.setVelocityConversionFactor(ENCODER_VELOCITY_COEFFICIENT);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeFeedRadPerSec = intakeEncoder.getVelocity();
        inputs.intakeAppliedVoltage = intakeMotor.getAppliedOutput() * 12;
        inputs.intakeCurrentDrawAmps = intakeMotor.getOutputCurrent();
    }

    public void setVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    @Data
    @EqualsAndHashCode(callSuper = true)
    public static class HardwareConfig extends IntakeConfig.HardwareConfig {
        private PLGConfig motor;
        private CanCoderConfig encoder;

        public IntakeIO createIO() {
            return new IntakeIOComp(
                motor.create(),
                "rio"
            );
        }
    }
}
