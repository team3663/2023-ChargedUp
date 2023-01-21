package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon implements GyroIO {
    private final PigeonIMU pigeon;

    private final double[] yawPitchRollDegrees = new double[3];

    public GyroIOPigeon(int id) {
        this.pigeon = new PigeonIMU(id);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        pigeon.getYawPitchRoll(yawPitchRollDegrees);

        inputs.yawRadians = Units.degreesToRadians(yawPitchRollDegrees[0]);
        inputs.pitchRadians = Units.degreesToRadians(yawPitchRollDegrees[1]);
        inputs.rollRadians = Units.degreesToRadians(yawPitchRollDegrees[2]);
    }
}
