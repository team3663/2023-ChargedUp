package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDrivetrainCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier xVelocitySupplier;
    private final DoubleSupplier yVelocitySupplier;
    private final DoubleSupplier angularVelocitySupplier;

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
    public void execute() {
        ChassisSpeeds chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocitySupplier.getAsDouble(),
                yVelocitySupplier.getAsDouble(),
                angularVelocitySupplier.getAsDouble(),
                drivetrain.getPose().getRotation()
        );

        drivetrain.setTargetChassisVelocity(chassisVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setTargetChassisVelocity(new ChassisSpeeds());
    }
}
