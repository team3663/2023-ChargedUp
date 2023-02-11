package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public double shoulderAngleRad;
        public double shoulderAngularVelRadPerSec;
        public double shoulderCurrentDrawAmps;
        public double shoulderAppliedVoltage;

        public double elbowAngleRad;
        public double elbowAngularVelRadPerSec;
        public double elbowCurrentDrawAmps;
        public double elbowAppliedVoltage;

        public double wristAngleRad;
        public double wristAngularVelRadPerSec;
        public double wristCurrentDrawAmps;
        public double wristAppliedVoltage;
    }

    default void updateInputs(ArmIOInputs inputs) {
    }

    default void setShoulderVoltage(double volts) {
    }

    default void setElbowVoltage(double volts) {
    }

    default void setWristVoltage(double volts) {
    }
}
