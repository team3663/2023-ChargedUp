package frc.robot.utility.config;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.JsonUnits;
import lombok.Data;

@Data
public final class CanCoderConfig {
    private int id;
    private String canBus = "rio";
    @JsonUnits(JsonUnits.Unit.DEGREES)
    private double offset;

    public CANCoder create() {
        WPI_CANCoder encoder = new WPI_CANCoder(id, canBus);
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.magnetOffsetDegrees = Units.radiansToDegrees(offset);
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorDirection = false;

        encoder.configAllSettings(config);

        // Workaround so that we always read a valid angle from the encoder.
        // Avoid using Thread.sleep and replace with an actual way to check if the encoder has received valid data
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            System.err.println("OOPS");
        }

        return encoder;
    }
}
