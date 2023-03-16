package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final String name;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModule(String name, SwerveModuleIO io) {
        this.name = name;
        this.io = io;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drivetrain/" + name, inputs);
    }

    public void setTargetState(SwerveModuleState state) {
        double currentAngleRad = inputs.steerAngleRad;
        double targetAngleRad = MathUtil.inputModulus(state.angle.getRadians(), 0.0, 2.0 * Math.PI);
        double absoluteAngleRad = MathUtil.inputModulus(currentAngleRad, 0.0, 2.0 * Math.PI);
        double errorRad = MathUtil.inputModulus(targetAngleRad - absoluteAngleRad, -Math.PI, Math.PI);

        double setpointRad = currentAngleRad + errorRad;

        Logger.getInstance().recordOutput("Drivetrain/" + name + "/TargetAngleRad", setpointRad);
        Logger.getInstance().recordOutput("Drivetrain/" + name + "/TargetVelocityMetersPerSec", state.speedMetersPerSecond);

        io.setTargetSteerAngle(setpointRad);
        io.setTargetDriveVelocity(state.speedMetersPerSecond);
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                inputs.drivePositionMeters,
                new Rotation2d(inputs.steerAngleRad)
        );
    }
}
