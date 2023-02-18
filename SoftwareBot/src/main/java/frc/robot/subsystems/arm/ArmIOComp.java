package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

public class ArmIOComp implements ArmIO {
    private static final double SHOULDER_OFFSET_DEG = 0;
    private static final double ELBOW_OFFSET_DEG = 0;
    private static final double WRIST_OFFSET_RAD = 0;

    private static final double SHOULDER_CURRENT_LIMIT = 40;
    private static final double ELBOW_CURRENT_LIMIT = 40;
    private static final double WRIST_CURRENT_LIMIT = 40;

    private final CANSparkMax shoulderMotor;
    private final TalonFX elbowMotor1;
    private final TalonFX elbowMotor2;
    private final TalonFX wristMotor;

    private final CANCoder shoulderEncoder;
    private final CANCoder elbowEncoder;
    private final CANCoder wristEncoder;

    public ArmIOComp(int shoulderMotorId, int shoulderEncoderId, int elbowMotor1Id, int elbowMotor2Id,
            int elbowEncoderId, int wristMotorId, int wristEncoderId) {
        this(shoulderMotorId, shoulderEncoderId, elbowMotor1Id, elbowMotor2Id, elbowEncoderId, wristMotorId,
                wristEncoderId, "rio");
    }

    public ArmIOComp(int shoulderMotorId, int shoulderEncoderId, int elbowMotor1Id, int elbowMotor2Id,
            int elbowEncoderId, int wristMotorId, int wristEncoderId, String canBusName) {
        this(new CANSparkMax(shoulderMotorId, MotorType.kBrushless), new CANCoder(elbowEncoderId),
                new TalonFX(elbowMotor1Id, canBusName), new TalonFX(elbowMotor2Id, canBusName),
                new CANCoder(elbowEncoderId), new TalonFX(wristMotorId, canBusName), new CANCoder(wristEncoderId));
    }

    public ArmIOComp(CANSparkMax shoulderMotor, CANCoder shoulderEncoder, TalonFX elbowMotor1, TalonFX elbowMotor2,
            CANCoder elbowEncoder, TalonFX wristMotor, CANCoder wristEncoder) {
        this.shoulderMotor = shoulderMotor;
        this.elbowMotor1 = elbowMotor1;
        this.elbowMotor2 = elbowMotor2;
        this.wristMotor = wristMotor;

        this.shoulderEncoder = shoulderEncoder;
        this.elbowEncoder = elbowEncoder;
        this.wristEncoder = wristEncoder;

        shoulderMotor.setSmartCurrentLimit((int) SHOULDER_CURRENT_LIMIT);
        shoulderMotor.enableVoltageCompensation(12);
        shoulderMotor.setInverted(true);

        TalonFXConfiguration elbowConfig = new TalonFXConfiguration();
        elbowConfig.supplyCurrLimit.currentLimit = ELBOW_CURRENT_LIMIT;
        elbowConfig.supplyCurrLimit.enable = true;
        elbowConfig.voltageCompSaturation = 12;

        elbowMotor1.configAllSettings(elbowConfig);
        elbowMotor1.enableVoltageCompensation(true);
        elbowMotor1.setNeutralMode(NeutralMode.Brake);
        elbowMotor1.setInverted(TalonFXInvertType.CounterClockwise);

        elbowMotor2.configAllSettings(elbowConfig);
        elbowMotor2.enableVoltageCompensation(true);
        elbowMotor2.setNeutralMode(NeutralMode.Brake);
        elbowMotor2.setInverted(TalonFXInvertType.CounterClockwise);

        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        wristConfig.supplyCurrLimit.currentLimit = WRIST_CURRENT_LIMIT;
        wristConfig.supplyCurrLimit.enable = true;
        wristConfig.voltageCompSaturation = 12;

        wristMotor.configAllSettings(wristConfig);
        wristMotor.enableVoltageCompensation(true);
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setInverted(TalonFXInvertType.CounterClockwise);

        // Remember that if the motor is inverted the encoder must be as well. If issues arise, this is the first place to look.
        CANCoderConfiguration shoulderEncoderConfig = new CANCoderConfiguration();
        shoulderEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        shoulderEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        shoulderEncoderConfig.magnetOffsetDegrees = SHOULDER_OFFSET_DEG;
        shoulderEncoderConfig.sensorDirection = false;
        shoulderEncoder.configAllSettings(shoulderEncoderConfig);

        CANCoderConfiguration elbowEncoderConfig = new CANCoderConfiguration();
        elbowEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        elbowEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        elbowEncoderConfig.magnetOffsetDegrees = ELBOW_OFFSET_DEG;
        elbowEncoderConfig.sensorDirection = false;
        elbowEncoder.configAllSettings(elbowEncoderConfig);

        CANCoderConfiguration wristEncoderConfig = new CANCoderConfiguration();
        wristEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        wristEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        wristEncoderConfig.magnetOffsetDegrees = WRIST_OFFSET_RAD;
        wristEncoderConfig.sensorDirection = false;
        wristEncoder.configAllSettings(wristEncoderConfig);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.shoulderAngleRad = shoulderEncoder.getAbsolutePosition();
        inputs.shoulderAngularVelRadPerSec = Units.degreesToRadians(shoulderEncoder.getVelocity());
        inputs.shoulderCurrentDrawAmps = shoulderMotor.getOutputCurrent();
        inputs.shoulderAppliedVoltage = shoulderMotor.getAppliedOutput() * 12;

        inputs.elbowAngleRad = elbowEncoder.getAbsolutePosition();
        inputs.elbowAngularVelRadPerSec = Units.degreesToRadians(elbowEncoder.getVelocity());
        inputs.elbowCurrentDrawAmps = (elbowMotor1.getMotorOutputVoltage() + elbowMotor2.getMotorOutputVoltage()) / 2;
        inputs.elbowAppliedVoltage = (elbowMotor1.getSupplyCurrent() + elbowMotor2.getSupplyCurrent()) / 2;

        inputs.wristAngleRad = wristEncoder.getAbsolutePosition();
        inputs.wristAngularVelRadPerSec = Units.degreesToRadians(wristEncoder.getVelocity());
        inputs.wristCurrentDrawAmps = wristMotor.getMotorOutputVoltage();
        inputs.wristAppliedVoltage = wristMotor.getSupplyCurrent();
    }

    @Override
    public void setShoulderVoltage(double volts) {
        shoulderMotor.setVoltage(volts);
    }

    @Override
    public void setElbowVoltage(double volts) {
        elbowMotor1.set(TalonFXControlMode.PercentOutput, volts / 12);
        elbowMotor2.set(TalonFXControlMode.PercentOutput, volts / 12);
    }

    @Override
    public void setWristVoltage(double volts) {
        wristMotor.set(TalonFXControlMode.PercentOutput, volts / 12);
    }

    public ArmState getArmState() {
        return new ArmState(Units.degreesToRadians(shoulderEncoder.getAbsolutePosition()),
                Units.degreesToRadians(elbowEncoder.getAbsolutePosition()),
                Units.degreesToRadians(wristEncoder.getAbsolutePosition()));
    }
}
