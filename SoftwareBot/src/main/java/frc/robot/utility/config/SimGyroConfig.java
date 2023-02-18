package frc.robot.utility.config;

import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import lombok.Data;
import lombok.EqualsAndHashCode;

@Data
@EqualsAndHashCode(callSuper = true)
public class SimGyroConfig extends GyroConfig {
    @Override
    public GyroIO createIO() {
        return new GyroIOSim();
    }
}
