package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public double pitchRadians;
        public double yawRadians;
        public double rollRadians;
    }

    default void updateInputs(GyroIOInputs inputs) {
    }
}
