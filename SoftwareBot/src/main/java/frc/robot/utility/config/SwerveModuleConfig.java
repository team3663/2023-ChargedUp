package frc.robot.utility.config;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import lombok.Data;

@Data
@JsonTypeInfo(use = JsonTypeInfo.Id.NAME)
@JsonSubTypes({@JsonSubTypes.Type(value = SdsMk4SwerveModuleConfig.class, name = "SdsMk4"),
        @JsonSubTypes.Type(value = SimSwerveModuleConfig.class, name = "Sim")})
public abstract class SwerveModuleConfig {
    @Data
    @JsonTypeInfo(use = JsonTypeInfo.Id.NAME)
    @JsonSubTypes({@JsonSubTypes.Type(value = SdsMk4SwerveModuleConfig.HardwareConfig.class, name = "SdsMk4")})
    public abstract static class HardwareConfig {
    }

    public abstract SwerveModuleIO createIO(HardwareConfig hardwareConfig);
}
