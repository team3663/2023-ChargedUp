package frc.robot.utility.config;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;

import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOComp;
import frc.robot.subsystems.intake.IntakeSubsystem;
import lombok.Data;

@Data
public final class IntakeConfig {
    
    public IntakeSubsystem createSubsystem() {
        return new IntakeSubsystem(null);
    }

    @Data
    @JsonTypeInfo(use = JsonTypeInfo.Id.NAME)
    @JsonSubTypes({@JsonSubTypes.Type(value = IntakeIOComp.HardwareConfig.class, name = "Comp")})
    public abstract static class HardwareConfig {
        public abstract IntakeIO createIO();
    }
}