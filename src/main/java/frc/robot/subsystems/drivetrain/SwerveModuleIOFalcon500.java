package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.util.Units;

public class SwerveModuleIOFalcon500 implements SwerveModuleIO {
    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
    private static final double DRIVE_WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);

    /**
     * A coefficient that converts encoder ticks of the drive motor to meters traveled of the drive wheel.
     * <p>
     * {@code meters = ticks * DRIVE_POSITION_COEFFICIENT}
     * <p>
     * {@code ticks = meters / DRIVE_POSITION_COEFFICIENT}
     */
    private static final double DRIVE_POSITION_COEFFICIENT = (1.0 / 2048.0) * (1.0 / DRIVE_GEAR_RATIO) * (Math.PI * DRIVE_WHEEL_DIAMETER_METERS);
    private static final double DRIVE_VELOCITY_COEFFICIENT = DRIVE_POSITION_COEFFICIENT * (1000.0 / 100.0);

    private static final double STEER_GEAR_RATIO = (32.0 / 15.0) * (60.0 / 10.0);

    /**
     * A coefficient that converts encoder ticks of the steer motor to radians of the steering pulley of the module.
     * <p>
     * {@code radians = ticks * STEER_POSITION_COEFFICIENT}
     * <p>
     * {@code ticks = radians / STEER_POSITION_COEFFICIENT}
     */
    private static final double STEER_POSITION_COEFFICIENT = (1.0 / 2048.0) * (1.0 / STEER_GEAR_RATIO) * (2.0 * Math.PI);
    private static final double STEER_VELOCITY_COEFFICIENT = STEER_POSITION_COEFFICIENT * (1000.0 / 100.0);

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANCoder steerEncoder;

    public SwerveModuleIOFalcon500(int driveMotorId, int steerMotorId, int steerEncoderId, double steerOffsetRad) {
        this(driveMotorId, steerMotorId, steerEncoderId, "rio", steerOffsetRad);
    }

    public SwerveModuleIOFalcon500(int driveMotorId, int steerMotorId, int steerEncoderId, String canBusName, double steerOffsetRad) {
        this(new TalonFX(driveMotorId, canBusName), new TalonFX(steerMotorId, canBusName), new CANCoder(steerEncoderId, canBusName), steerOffsetRad);
    }

    public SwerveModuleIOFalcon500(TalonFX driveMotor, TalonFX steerMotor, CANCoder steerEncoder, double steerOffsetRad) {
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

        steerMotor.configAllSettings(steerConfig, 500);
        steerMotor.enableVoltageCompensation(true);
        steerMotor.setNeutralMode(NeutralMode.Coast);
        steerMotor.setInverted(TalonFXInvertType.CounterClockwise);

        // Configure the steer encoder
        CANCoderConfiguration steerEncoderConfig = new CANCoderConfiguration();
        steerEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        steerEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        steerEncoderConfig.magnetOffsetDegrees = Units.radiansToDegrees(steerOffsetRad);
        steerEncoderConfig.sensorDirection = false;

        steerEncoder.configAllSettings(steerEncoderConfig);

        // Workaround so that we always read a valid angle from the steer encoder when setting up the steer motor.
        // Avoid using Thread.sleep and replace with an actual way to check if the steer encoder has received valid data
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            System.err.println("OOPS");
        }

        // Synchronize steer motor encoder & steer absolute encoder for PID control
        steerMotor.setSelectedSensorPosition(Units.degreesToRadians(steerEncoder.getAbsolutePosition()) / STEER_POSITION_COEFFICIENT);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveMotor.getSelectedSensorPosition() * DRIVE_POSITION_COEFFICIENT;
        inputs.driveVelocityMetersPerSec = driveMotor.getSelectedSensorVelocity() * DRIVE_VELOCITY_COEFFICIENT;
        inputs.driveAppliedVolts = driveMotor.getMotorOutputVoltage();
        inputs.driveCurrentDrawAmps = driveMotor.getSupplyCurrent();

        inputs.steerAngleRad = steerMotor.getSelectedSensorPosition() * STEER_POSITION_COEFFICIENT;
        inputs.steerAngularVelocityRadPerSec = steerMotor.getSelectedSensorVelocity() * STEER_VELOCITY_COEFFICIENT;
        inputs.steerAbsoluteAngleRad = Units.degreesToRadians(steerEncoder.getAbsolutePosition());
        inputs.steerAppliedVolts = steerMotor.getMotorOutputVoltage();
        inputs.steerCurrentDrawAmps = steerMotor.getSupplyCurrent();
    }

    @Override
    public void setTargetDriveVelocity(double velocityMetersPerSec) {
        driveMotor.set(TalonFXControlMode.Velocity, velocityMetersPerSec / DRIVE_VELOCITY_COEFFICIENT);
    }

    @Override
    public void setTargetSteerAngle(double angleRad) {
        steerMotor.set(TalonFXControlMode.Position, angleRad / STEER_POSITION_COEFFICIENT);
    }
}
