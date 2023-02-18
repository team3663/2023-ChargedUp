package frc.robot.utility.config;

import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSdsMk4;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.EqualsAndHashCode;
import lombok.Getter;

@Data
@EqualsAndHashCode(callSuper = true)
public class SdsMk4SwerveModuleConfig extends SwerveModuleConfig {
    private GearRatio gearRatio = GearRatio.L3;

    @Override
    public SwerveModuleIO createIO(SwerveModuleConfig.HardwareConfig hardwareConfig) {
        if (!(hardwareConfig instanceof HardwareConfig)) {
            throw new IllegalArgumentException("Hardware config for a Mk4 module must be a Mk4 hardware configuration");
        }

        HardwareConfig sdsHardwareConfig = (HardwareConfig) hardwareConfig;

        return new SwerveModuleIOSdsMk4(gearRatio, sdsHardwareConfig.driveMotor.create(), sdsHardwareConfig.steerMotor.create(), sdsHardwareConfig.steerEncoder.create());
    }

    @Data
    @EqualsAndHashCode(callSuper = true)
    public static class HardwareConfig extends SwerveModuleConfig.HardwareConfig {
        private Falcon500Config driveMotor;
        private Falcon500Config steerMotor;
        private CanCoderConfig steerEncoder;
    }

    @AllArgsConstructor
    public enum GearRatio {
        L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
        L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
        L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
        L4((48.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0));

        @Getter
        private final double reduction;
    }
}
