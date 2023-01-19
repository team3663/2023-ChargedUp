package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        public double drivePositionMeters;
        public double driveVelocityMetersPerSec;
        public double driveAppliedVolts;
        public double driveCurrentDrawAmps;

        public double steerAngleRad;
        public double steerAngularVelocityRadPerSec;
        public double steerAbsoluteAngleRad;
        public double steerAppliedVolts;
        public double steerCurrentDrawAmps;
    }

    default void updateInputs(SwerveModuleIOInputs inputs) {
    }

    default void setTargetDriveVelocity(double velocityMetersPerSec) {
    }

    default void setTargetSteerAngle(double angleRad) {
    }
}
