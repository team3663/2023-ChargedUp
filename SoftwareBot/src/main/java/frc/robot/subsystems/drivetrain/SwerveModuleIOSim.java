package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    private final FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025);
    private final FlywheelSim steerSim = new FlywheelSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004096955);

    private double steerRelativePositionRad = 0;
    private double steerAbsolutePositionRad = Math.random() * 2.0 * Math.PI;

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 12.0 / Units.feetToMeters(16.5), 12.0 / Units.feetToMeters(60.0));
    private final PIDController driveFeedback = new PIDController(10, 0, 0);
    private final PIDController steerFeedback = new PIDController(10, 0, 0);

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        double driveAppliedVolts = driveFeedforward.calculate(driveFeedback.getSetpoint()) + driveFeedback.calculate(inputs.driveVelocityMetersPerSec);
        driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
        double steerAppliedVolts = steerFeedback.calculate(inputs.steerAngleRad);
        steerAppliedVolts = MathUtil.clamp(steerAppliedVolts, -12.0, 12.0);

        driveSim.setInputVoltage(driveAppliedVolts);
        steerSim.setInputVoltage(steerAppliedVolts);

        driveSim.update(Robot.defaultPeriodSecs);
        steerSim.update(Robot.defaultPeriodSecs);

        double angleDiffRad = steerSim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs;
        steerRelativePositionRad += angleDiffRad;
        steerAbsolutePositionRad += angleDiffRad;

        steerAbsolutePositionRad = MathUtil.inputModulus(steerAbsolutePositionRad, 0, 2 * Math.PI);

        inputs.drivePositionMeters += (driveSim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs) * WHEEL_RADIUS_METERS;
        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * WHEEL_RADIUS_METERS;
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentDrawAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.steerAngleRad = steerRelativePositionRad;
        inputs.steerAngularVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = steerAppliedVolts;
        inputs.steerCurrentDrawAmps = Math.abs(steerSim.getCurrentDrawAmps());

        inputs.steerAbsoluteAngleRad = steerAbsolutePositionRad;
    }

    @Override
    public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
        driveFeedback.setSetpoint(targetDriveVelocityMetersPerSec);
    }

    @Override
    public void setTargetSteerAngle(double targetSteerAngleRad) {
        steerFeedback.setSetpoint(targetSteerAngleRad);
    }
}
