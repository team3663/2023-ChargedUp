package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.config.CanCoderConfig;
import frc.robot.utility.config.Falcon500Config;
import frc.robot.utility.config.SwerveModuleConfig;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.EqualsAndHashCode;
import lombok.Getter;

public class SwerveModuleIOSdsMk4 implements SwerveModuleIO {
    private static final double WHEEL_NOMINAL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    // This value accounts for the wearing down of the tread over the course of a competition, as well as the "squish factor" between
    // the wheels and the carpet. Function may be added down the line to dynamically adjust this value in the field.
    private static final double TREADWEAR = Units.inchesToMeters(0.125);
    private static final double WHEEL_DIAMETER_METERS = WHEEL_NOMINAL_DIAMETER_METERS - TREADWEAR;
    private static final double STEER_REDUCTION = (32.0 / 15.0) * (60.0 / 10.0);

    private final double drivePositionCoefficient;
    private final double driveVelocityCoefficient;
    private final double driveMaximumVelocity;

    private final double steerPositionCoefficient;
    private final double steerVelocityCoefficient;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANCoder steerEncoder;

    public SwerveModuleIOSdsMk4(
            Config.GearRatio gearRatio,
            TalonFX driveMotor,
            TalonFX steerMotor,
            CANCoder steerEncoder
    ) {
        drivePositionCoefficient = (1.0 / 2048.0) * (1.0 / gearRatio.getReduction()) * (Math.PI * WHEEL_DIAMETER_METERS);
        driveVelocityCoefficient = drivePositionCoefficient * 10.0;
        driveMaximumVelocity = DCMotor.getFalcon500(1).freeSpeedRadPerSec * (2048.0 / 10.0) * (1.0 / (2.0 * Math.PI)) * driveVelocityCoefficient;

        steerPositionCoefficient = (1.0 / 2048.0) * (1.0 / STEER_REDUCTION) * (2.0 * Math.PI);
        steerVelocityCoefficient = steerPositionCoefficient * 10.0;

        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;

        // Configure the drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        driveConfig.slot0.kP = 0.04;
        driveConfig.slot0.kF = 0.045787;
        driveConfig.supplyCurrLimit.currentLimit = 40.0;
        driveConfig.supplyCurrLimit.enable = true;
        driveConfig.voltageCompSaturation = 12.0;

        driveMotor.configAllSettings(driveConfig);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(TalonFXInvertType.Clockwise);

        // Configure the steer motor
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        steerConfig.slot0.kP = 0.5;
        steerConfig.slot0.kD = 0.4;
        steerConfig.supplyCurrLimit.currentLimit = 10.0;
        steerConfig.supplyCurrLimit.enable = true;
        steerConfig.voltageCompSaturation = 12.0;

        steerMotor.configAllSettings(steerConfig);
        steerMotor.enableVoltageCompensation(true);
        steerMotor.setNeutralMode(NeutralMode.Coast);
        steerMotor.setInverted(TalonFXInvertType.CounterClockwise);

        // Workaround so that we always read a valid angle from the steer encoder when setting up the steer motor.
        // Avoid using Thread.sleep and replace with an actual way to check if the steer encoder has received valid data
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            System.err.println("OOPS");
        }

        // Synchronize steer motor encoder & steer absolute encoder for PID control
        steerMotor.setSelectedSensorPosition(Units.degreesToRadians(steerEncoder.getAbsolutePosition()) / steerPositionCoefficient);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveMotor.getSelectedSensorPosition() * drivePositionCoefficient;
        inputs.driveVelocityMetersPerSec = driveMotor.getSelectedSensorVelocity() * driveVelocityCoefficient;
        inputs.driveAppliedVolts = driveMotor.getMotorOutputVoltage();
        inputs.driveCurrentDrawAmps = driveMotor.getSupplyCurrent();

        inputs.steerAngleRad = steerMotor.getSelectedSensorPosition() * steerPositionCoefficient;
        inputs.steerAngularVelocityRadPerSec = steerMotor.getSelectedSensorVelocity() * steerVelocityCoefficient;
        inputs.steerAbsoluteAngleRad = Units.degreesToRadians(steerEncoder.getAbsolutePosition());
        inputs.steerAppliedVolts = steerMotor.getMotorOutputVoltage();
        inputs.steerCurrentDrawAmps = steerMotor.getSupplyCurrent();
    }

    @Override
    public void setTargetDriveVelocity(double velocityMetersPerSec) {
        driveMotor.set(TalonFXControlMode.PercentOutput, velocityMetersPerSec / driveMaximumVelocity);
    }

    @Override
    public void setTargetSteerAngle(double angleRad) {
        steerMotor.set(TalonFXControlMode.Position, angleRad / steerPositionCoefficient);
    }

    @Data
    @EqualsAndHashCode(callSuper = true)
    public static class Config extends SwerveModuleConfig {
        private GearRatio gearRatio = GearRatio.L3;

        @Override
        public SwerveModuleIO createIO(SwerveModuleConfig.HardwareConfig hardwareConfig) {
            if (!(hardwareConfig instanceof SwerveModuleIOSdsMk4.HardwareConfig)) {
                throw new IllegalArgumentException("Hardware config for a Mk4 module must be a Mk4 hardware configuration");
            }

            SwerveModuleIOSdsMk4.HardwareConfig sdsHardwareConfig = (SwerveModuleIOSdsMk4.HardwareConfig) hardwareConfig;

            return new SwerveModuleIOSdsMk4(gearRatio, sdsHardwareConfig.getDriveMotor().create(), sdsHardwareConfig.getSteerMotor().create(), sdsHardwareConfig.getSteerEncoder().create());
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

    @Data
    @EqualsAndHashCode(callSuper = true)
    public static class HardwareConfig extends SwerveModuleConfig.HardwareConfig {
        private Falcon500Config driveMotor;
        private Falcon500Config steerMotor;
        private CanCoderConfig steerEncoder;
    }
}
