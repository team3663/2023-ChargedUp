package frc.robot.utility.config;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSdsMk4;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import lombok.Data;

@Data
@JsonTypeInfo(use = JsonTypeInfo.Id.NAME)
@JsonSubTypes({@JsonSubTypes.Type(value = SwerveModuleIOSdsMk4.Config.class, name = "SdsMk4"),
        @JsonSubTypes.Type(value = SwerveModuleIOSim.Config.class, name = "Sim")})
public abstract class SwerveModuleConfig {
    @Data
    @JsonTypeInfo(use = JsonTypeInfo.Id.NAME)
    @JsonSubTypes({@JsonSubTypes.Type(value = SwerveModuleIOSdsMk4.HardwareConfig.class, name = "SdsMk4")})
    public abstract static class HardwareConfig {
    }

    public abstract SwerveModuleIO createIO(HardwareConfig hardwareConfig);
}
