package frc.robot.utility.config;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;

import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOComp;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import lombok.Data;

@Data
public class ArmConfig {
    private HardwareConfig hardware;

    public ArmSubsystem createSubsystem() {
        var io = hardware == null ? new ArmIO() {} : hardware.createIO();
        return new ArmSubsystem(io);
    }

    @Data
    @JsonTypeInfo(use = JsonTypeInfo.Id.NAME)
    @JsonSubTypes({@JsonSubTypes.Type(value = ArmIOComp.HardwareConfig.class, name = "Comp"),
        @JsonSubTypes.Type(value = ArmIOSim.HardwareConfig.class, name = "Sim")})
    public abstract static class HardwareConfig {
        public abstract ArmIO createIO();
    }
}
