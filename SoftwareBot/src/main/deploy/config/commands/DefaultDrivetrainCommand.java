package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDrivetrainCommand extends CommandBase {
    private static final double MAX_LINEAR_ACCELERATION_METERS_PER_SEC = 10;
    private static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC = 50;

    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier xVelocitySupplier;
    private final DoubleSupplier yVelocitySupplier;
    private final DoubleSupplier angularVelocitySupplier;
    private final SlewRateLimiter xVelocityLimiter = new SlewRateLimiter(MAX_LINEAR_ACCELERATION_METERS_PER_SEC);
    private final SlewRateLimiter yVelocityLimiter = new SlewRateLimiter(MAX_LINEAR_ACCELERATION_METERS_PER_SEC);
    private final SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(MAX_ANGULAR_ACCELERATION_RAD_PER_SEC);
    
    public DefaultDrivetrainCommand(DrivetrainSubsystem drivetrain,
                                    DoubleSupplier xVelocitySupplier,
                                    DoubleSupplier yVelocitySupplier,
                                    DoubleSupplier angularVelocitySupplier) {
        this.drivetrain = drivetrain;
        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xVelocityLimiter.reset(xVelocitySupplier.getAsDouble());
        yVelocityLimiter.reset(yVelocitySupplier.getAsDouble());
        angularVelocityLimiter.reset(angularVelocitySupplier.getAsDouble());
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocityLimiter.calculate(xVelocitySupplier.getAsDouble()),
            yVelocityLimiter.calculate(yVelocitySupplier.getAsDouble()),
            angularVelocityLimiter.calculate(angularVelocitySupplier.getAsDouble()),
            drivetrain.getPose().getRotation()
        );

        drivetrain.setTargetChassisVelocity(chassisVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setTargetChassisVelocity(new ChassisSpeeds());
    }
}
