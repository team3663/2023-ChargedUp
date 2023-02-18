package frc.robot.utility.config;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import frc.robot.subsystems.drivetrain.GyroIO;
import lombok.Data;

@Data
@JsonTypeInfo(use = JsonTypeInfo.Id.NAME)
@JsonSubTypes({@JsonSubTypes.Type(value = Pigeon2Config.class, name = "Pigeon2"),
@JsonSubTypes.Type(value = SimGyroConfig.class, name = "Sim")})
public abstract class GyroConfig {

    public abstract GyroIO createIO();
}
