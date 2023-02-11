package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ArmIOSim implements ArmIO {
    private static final double ARM_CG_MOMENT_OF_INERTIA = 0.5;
    private static final double ARM_MASS = 6.5;

    private static final double FOREARM_CG_MOMENT_OF_INERTIA = 0.32516064;
    private static final double FOREARM_MASS = 4.5;

    private static final double INTAKE_CG_MOMENT_OF_INERTIA = 0.2;
    private static final double INTAKE_MASS = Units.lbsToKilograms(5.0);

    private static final Translation2d SHOULDER_ARM_CG_OFFSET = new Translation2d(Units.inchesToMeters(22.0), 0.0);
    private static final Translation2d SHOULDER_ELBOW_OFFSET = new Translation2d(Units.inchesToMeters(44.0), 0.0);
    private static final DCMotor SHOULDER_MOTOR = DCMotor.getFalcon500(2).withReduction(100.0);
    private static final double SHOULDER_CURRENT_LIMIT = 20.0;

    private static final Translation2d ELBOW_FOREARM_CG_OFFSET = new Translation2d(Units.inchesToMeters(18.0), 0.0);
    private static final Translation2d ELBOW_WRIST_OFFSET = new Translation2d(Units.inchesToMeters(36.0), 0.0);
    private static final DCMotor ELBOW_MOTOR = DCMotor.getFalcon500(2).withReduction(100.0);
    private static final double ELBOW_CURRENT_LIMIT = 10.0;

    private static final Translation2d WRIST_INTAKE_CG_OFFSET = new Translation2d(Units.inchesToMeters(4.0), 0.0);
    private static final DCMotor WRIST_MOTOR = DCMotor.getFalcon500(1).withReduction(50.0);
    private static final double WRIST_CURRENT_LIMIT = 10.0;

    private double shoulderVolts = 0.0;
    private double elbowVolts = 0.0;
    private double wristVolts = 0.0;

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        Matrix<N6, N1> x = VecBuilder.fill(
                inputs.shoulderAngleRad,
                inputs.shoulderAngularVelRadPerSec,
                inputs.elbowAngleRad,
                inputs.elbowAngularVelRadPerSec,
                inputs.wristAngleRad,
                inputs.wristAngularVelRadPerSec
        );

        Matrix<N3, N1> u = VecBuilder.fill(
                shoulderVolts,
                elbowVolts,
                wristVolts
        );

        Matrix<N6, N1> xNext = NumericalIntegration.rk4(ArmIOSim::calculateDynamics, x, u, 0.02);

        inputs.shoulderAngleRad = xNext.get(0, 0);
        inputs.shoulderAngularVelRadPerSec = xNext.get(1, 0);
        inputs.shoulderAppliedVoltage = shoulderVolts;
        inputs.shoulderCurrentDrawAmps = Math.min(Math.abs(SHOULDER_MOTOR.getCurrent(inputs.shoulderAngularVelRadPerSec, inputs.shoulderAppliedVoltage)), SHOULDER_CURRENT_LIMIT);

        inputs.elbowAngleRad = xNext.get(2, 0);
        inputs.elbowAngularVelRadPerSec = xNext.get(3, 0);
        inputs.elbowAppliedVoltage = elbowVolts;
        inputs.elbowCurrentDrawAmps = Math.min(Math.abs(ELBOW_MOTOR.getCurrent(inputs.elbowAngularVelRadPerSec, inputs.elbowAppliedVoltage)), ELBOW_CURRENT_LIMIT);

        inputs.wristAngleRad = xNext.get(4, 0);
        inputs.wristAngularVelRadPerSec = xNext.get(5, 0);
        inputs.wristAppliedVoltage = wristVolts;
        inputs.wristCurrentDrawAmps = Math.min(Math.abs(WRIST_MOTOR.getCurrent(inputs.wristAngularVelRadPerSec, inputs.wristAppliedVoltage)), WRIST_CURRENT_LIMIT);
    }

    private static Matrix<N6, N1> calculateDynamics(Matrix<N6, N1> x, Matrix<N3, N1> u) {
        double shoulderAngleRad = x.get(0, 0);
        double elbowAngleRad = x.get(2, 0);
        double wristAngleRad = x.get(4, 0);

        double shoulderIntakeCgDistance = WRIST_INTAKE_CG_OFFSET.rotateBy(new Rotation2d(wristAngleRad)).plus(ELBOW_WRIST_OFFSET)
                .rotateBy(new Rotation2d(elbowAngleRad)).plus(SHOULDER_ELBOW_OFFSET).getNorm();
        double shoulderForearmCgDistance = ELBOW_FOREARM_CG_OFFSET.rotateBy(new Rotation2d(elbowAngleRad)).plus(SHOULDER_ELBOW_OFFSET).getNorm();
        double shoulderArmCgDistance = SHOULDER_ARM_CG_OFFSET.getNorm();
        double shoulderMomentOfInertia = (ARM_CG_MOMENT_OF_INERTIA + ARM_MASS * shoulderArmCgDistance * shoulderArmCgDistance)
                + (FOREARM_CG_MOMENT_OF_INERTIA + FOREARM_MASS * shoulderForearmCgDistance * shoulderForearmCgDistance)
                + (INTAKE_CG_MOMENT_OF_INERTIA + INTAKE_MASS * shoulderIntakeCgDistance * shoulderIntakeCgDistance);

        Translation2d elbowIntakeCgOffset = WRIST_INTAKE_CG_OFFSET.rotateBy(new Rotation2d(wristAngleRad)).plus(ELBOW_WRIST_OFFSET);

        double elbowIntakeCgDistance = elbowIntakeCgOffset.getNorm();
        double elbowForearmCgDistance = ELBOW_FOREARM_CG_OFFSET.getNorm();
        double elbowMomentOfInertia = (FOREARM_CG_MOMENT_OF_INERTIA + FOREARM_MASS * elbowForearmCgDistance * elbowForearmCgDistance)
                + (INTAKE_CG_MOMENT_OF_INERTIA + INTAKE_MASS * elbowIntakeCgDistance * elbowIntakeCgDistance);
        Translation2d elbowCg = ELBOW_FOREARM_CG_OFFSET.times(FOREARM_MASS).plus(elbowIntakeCgOffset.times(INTAKE_MASS)).div(FOREARM_MASS + INTAKE_MASS)
                .rotateBy(new Rotation2d(Math.PI - elbowAngleRad + shoulderAngleRad));

        double wristIntakeCgDistance = WRIST_INTAKE_CG_OFFSET.getNorm();
        double wristMomentOfInertia = INTAKE_CG_MOMENT_OF_INERTIA + INTAKE_MASS * wristIntakeCgDistance * wristIntakeCgDistance;

        Matrix<N6, N1> xDot = new Matrix<>(Nat.N6(), Nat.N1());

        // Shoulder
        double shoulderAppliedVolts = u.get(0, 0);
        double shoulderAngularVelocity = x.get(1, 0);
        double shoulderCurrentDraw = MathUtil.clamp(SHOULDER_MOTOR.getCurrent(shoulderAngularVelocity, shoulderAppliedVolts), -SHOULDER_CURRENT_LIMIT, SHOULDER_CURRENT_LIMIT);
        double shoulderTorque = SHOULDER_MOTOR.getTorque(shoulderCurrentDraw);

        double shoulderAngularAcceleration = shoulderTorque / shoulderMomentOfInertia;
        xDot.set(0, 0, shoulderAngularVelocity);
        xDot.set(1, 0, shoulderAngularAcceleration);

        // Elbow
        double elbowAppliedVolts = u.get(1, 0);
        double elbowAngularVelocity = x.get(3, 0);
        double elbowCurrentDraw = MathUtil.clamp(ELBOW_MOTOR.getCurrent(elbowAngularVelocity, elbowAppliedVolts), -ELBOW_CURRENT_LIMIT, ELBOW_CURRENT_LIMIT);
        double elbowTorque = ELBOW_MOTOR.getTorque(elbowCurrentDraw);
        elbowTorque += elbowCg.getNorm() * 9.81 * elbowCg.getAngle().getCos();

        double elbowAngularAcceleration =  elbowTorque / elbowMomentOfInertia;
        xDot.set(2, 0, elbowAngularVelocity);
        xDot.set(3, 0, elbowAngularAcceleration);

        // Wrist
        double wristAppliedVolts = u.get(2, 0);
        double wristAngularVelocity = x.get(5, 0);
        double wristCurrentDraw = MathUtil.clamp(WRIST_MOTOR.getCurrent(wristAngularVelocity, wristAppliedVolts), -WRIST_CURRENT_LIMIT, WRIST_CURRENT_LIMIT);
        double wristTorque = WRIST_MOTOR.getTorque(wristCurrentDraw);

        Rotation2d intakeAngle = new Rotation2d(shoulderAngleRad + (Math.PI - elbowAngleRad) - wristAngleRad);
        wristTorque += wristIntakeCgDistance * 9.81 * intakeAngle.getCos();

        double wristAngularAcceleration = wristTorque / wristMomentOfInertia;

        xDot.set(4, 0, wristAngularVelocity);
        xDot.set(5, 0, wristAngularAcceleration);

        return xDot;
    }

    @Override
    public void setShoulderVoltage(double volts) {
        this.shoulderVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setElbowVoltage(double volts) {
        this.elbowVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setWristVoltage(double volts) {
        this.wristVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}
