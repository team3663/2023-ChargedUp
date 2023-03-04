package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.utility.config.ArmConfig;
import frc.robot.utility.config.CanCoderConfig;
import frc.robot.utility.config.Falcon500Config;
import frc.robot.utility.config.NeoConfig;
import lombok.Data;
import lombok.EqualsAndHashCode;

public class ArmIOComp implements ArmIO {
    private static final double SHOULDER_CURRENT_LIMIT = 40;
    private static final double ELBOW_CURRENT_LIMIT = 40;
    private static final double WRIST_CURRENT_LIMIT = 40;

    private final CANSparkMax shoulderMotor;
    private final TalonFX elbowMotor;
    private final TalonFX wristMotor;

    private final CANCoder shoulderEncoder;
    private final CANCoder elbowEncoder;
    private final CANCoder wristEncoder;

    public ArmIOComp(int shoulderMotorId, int shoulderEncoderId, int elbowMotorId,
            int elbowEncoderId, int wristMotorId, int wristEncoderId) {
        this(shoulderMotorId, shoulderEncoderId, elbowMotorId, elbowEncoderId, wristMotorId,
                wristEncoderId, "rio");
    }

    public ArmIOComp(int shoulderMotorId, int shoulderEncoderId, int elbowMotorId,
            int elbowEncoderId, int wristMotorId, int wristEncoderId, String canBusName) {
        this(new CANSparkMax(shoulderMotorId, MotorType.kBrushless), new CANCoder(elbowEncoderId),
                new TalonFX(elbowMotorId, canBusName), new CANCoder(elbowEncoderId),
                new TalonFX(wristMotorId, canBusName), new CANCoder(wristEncoderId));
    }

    public ArmIOComp(CANSparkMax shoulderMotor, CANCoder shoulderEncoder, TalonFX elbowMotor,
            CANCoder elbowEncoder, TalonFX wristMotor, CANCoder wristEncoder) {
        this.shoulderMotor = shoulderMotor;
        this.elbowMotor = elbowMotor;
        this.wristMotor = wristMotor;

        this.shoulderEncoder = shoulderEncoder;
        this.elbowEncoder = elbowEncoder;
        this.wristEncoder = wristEncoder;

        shoulderMotor.setSmartCurrentLimit((int) SHOULDER_CURRENT_LIMIT);
        shoulderMotor.enableVoltageCompensation(12);

        TalonFXConfiguration elbowConfig = new TalonFXConfiguration();
        elbowConfig.supplyCurrLimit.currentLimit = ELBOW_CURRENT_LIMIT;
        elbowConfig.supplyCurrLimit.enable = true;
        elbowConfig.voltageCompSaturation = 12;
        elbowConfig.neutralDeadband = 0.0;

        elbowMotor.configAllSettings(elbowConfig);
        elbowMotor.enableVoltageCompensation(true);
        elbowMotor.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        wristConfig.supplyCurrLimit.currentLimit = WRIST_CURRENT_LIMIT;
        wristConfig.supplyCurrLimit.enable = true;
        wristConfig.voltageCompSaturation = 12;

        wristMotor.configAllSettings(wristConfig);
        wristMotor.enableVoltageCompensation(true);
        wristMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.shoulderAngleRad = Units.degreesToRadians(shoulderEncoder.getAbsolutePosition());
        inputs.shoulderAngularVelRadPerSec = Units.degreesToRadians(shoulderEncoder.getVelocity());
        inputs.shoulderCurrentDrawAmps = shoulderMotor.getOutputCurrent();
        inputs.shoulderAppliedVoltage = shoulderMotor.getAppliedOutput() * 12;

        inputs.elbowAngleRad = Units.degreesToRadians(elbowEncoder.getAbsolutePosition());
        inputs.elbowAngularVelRadPerSec = Units.degreesToRadians(elbowEncoder.getVelocity());
        inputs.elbowCurrentDrawAmps = elbowMotor.getMotorOutputVoltage();
        inputs.elbowAppliedVoltage = elbowMotor.getSupplyCurrent();

        inputs.wristAngleRad = Units.degreesToRadians(wristEncoder.getAbsolutePosition());
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
        elbowMotor.set(TalonFXControlMode.PercentOutput, volts / 12);
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

    @Data
    @EqualsAndHashCode(callSuper = true)
    public static class HardwareConfig extends ArmConfig.HardwareConfig {
        private NeoConfig shoulderMotor;
        private CanCoderConfig shoulderEncoder;
        private Falcon500Config elbowMotor;
        private CanCoderConfig elbowEncoder;
        private Falcon500Config wristMotor;
        private CanCoderConfig wristEncoder;

        public ArmIO createIO() {
            return new ArmIOComp(
                shoulderMotor.create(),
                shoulderEncoder.create(),
                elbowMotor.create(),
                elbowEncoder.create(),
                wristMotor.create(),
                wristEncoder.create()
            );
        }
    }
}
