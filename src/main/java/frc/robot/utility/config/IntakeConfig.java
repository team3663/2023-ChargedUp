package frc.robot.utility.config;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;

import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOComp;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import lombok.Data;

@Data
public final class IntakeConfig {
    private HardwareConfig hardware;

    public IntakeSubsystem createSubsystem() {
        return new IntakeSubsystem(hardware.createIO());
    }

    @Data
    @JsonTypeInfo(use = JsonTypeInfo.Id.NAME)
    @JsonSubTypes({@JsonSubTypes.Type(value = IntakeIOComp.HardwareConfig.class, name = "Comp"),
        @JsonSubTypes.Type(value = IntakeIOSim.HardwareConfig.class, name = "Sim")})
    public abstract static class HardwareConfig {
        public abstract IntakeIO createIO();
    }
}