package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class DriveForwardCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final double velocity;
    private final long durationMs;

    private ChassisSpeeds chassisSpeeds;
    private long endTime;

    public DriveForwardCommand(DrivetrainSubsystem drivetrain, double velocity, long durationMs) {
        this.drivetrain = drivetrain;
        this.velocity = velocity;
        this.durationMs = durationMs;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

        endTime = System.currentTimeMillis() + durationMs;
        chassisSpeeds = new ChassisSpeeds(velocity, 0.0, 0.0);
        drivetrain.drive(chassisSpeeds);
    }

    @Override
    public void execute() {
    }


    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0,0,0));
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= endTime;
    }
}
