// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.utility.config.IntakeConfig;
import lombok.Data;
import lombok.EqualsAndHashCode;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
    public IntakeIOSim() {
    }

    public void updateInputs() {
    }

    public void setVoltage() {
    }

    public void setIdleMode() {
    }

    @Data
    @EqualsAndHashCode(callSuper = true)
    public static class HardwareConfig extends IntakeConfig.HardwareConfig {

        public IntakeIO createIO() {
            return new IntakeIOSim();
        }
    }
}
