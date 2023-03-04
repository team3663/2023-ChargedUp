// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class IntakeIOComp implements IntakeIO {

    // Converts RPM into rad/sec. This is a potential source of problems
    private final double ENCODER_VELOCITY_COEFFICIENT = 2 * Math.PI / 60;

    private final double INTAKE_CURRENT_LIMIT = 40;

    private final CANSparkMax intakeMotor;

    private final RelativeEncoder intakeEncoder;

    public IntakeIOComp(int intakeMotorId, int intakeEncoderId) {
        this(intakeMotorId, intakeEncoderId, "rio");
    }

    public IntakeIOComp(int intakeMotorId, int intakeEncoderId, String canBusName) {
        intakeMotor = new CANSparkMax(intakeMotorId, MotorType.kBrushed);
        intakeEncoder = intakeMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 45);

        intakeMotor.setSmartCurrentLimit((int) INTAKE_CURRENT_LIMIT);
        intakeMotor.enableVoltageCompensation(12);
        intakeMotor.setInverted(false);

        intakeEncoder.setVelocityConversionFactor(ENCODER_VELOCITY_COEFFICIENT);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeFeedRadPerSec = intakeEncoder.getVelocity();
        inputs.intakeAppliedVoltage = intakeMotor.getAppliedOutput() * 12;
        inputs.intakeCurrentDrawAmps = intakeMotor.getOutputCurrent();
    }

    public void setVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }
}
