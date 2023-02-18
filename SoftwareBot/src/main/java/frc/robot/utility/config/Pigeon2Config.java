package frc.robot.utility.config;

import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import lombok.Data;
import lombok.EqualsAndHashCode;

@Data
@EqualsAndHashCode(callSuper = true)
public class Pigeon2Config extends GyroConfig {
    private int id;
    private String canBus = "rio";

    @Override
    public GyroIO createIO() {
        return new GyroIOPigeon2(id, canBus);
    }
}
