// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility.config;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import lombok.Data;

/** Add your docs here. */
@Data
public final class NeoConfig {
    private int id;
    private boolean inverted = false;

    public CANSparkMax create() {
        CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);
        motor.setInverted(inverted);

        return motor;
    }
}
