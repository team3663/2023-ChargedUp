package frc.robot.utility.config;

import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;

public class SimSwerveModuleConfig extends SwerveModuleConfig {
    @Override
    public SwerveModuleIO createIO(HardwareConfig hardwareConfig) {
        if (hardwareConfig != null) {
            throw new IllegalArgumentException("Hardware config must be null for simulation");
        }

        return new SwerveModuleIOSim();
    }
}
