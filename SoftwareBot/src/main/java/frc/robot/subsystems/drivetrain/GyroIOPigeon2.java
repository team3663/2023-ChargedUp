package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    private final double[] yawPitchRollDegrees = new double[3];

    public GyroIOPigeon2(int id) {
        this(id, "rio");
    }

    public GyroIOPigeon2(int id, String canBus) {
        this.pigeon = new Pigeon2(id, canBus);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        pigeon.getYawPitchRoll(yawPitchRollDegrees);

        inputs.yawRadians = Units.degreesToRadians(yawPitchRollDegrees[0]);
        inputs.pitchRadians = Units.degreesToRadians(yawPitchRollDegrees[1]);
        inputs.rollRadians = Units.degreesToRadians(yawPitchRollDegrees[2]);
    }
}
