// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class AutoBalanceCommand extends CommandBase {
    private static final long LEVEL_TIME = 1000;
    private static final double TILT_TOLERANCE = Units.degreesToRadians(2);
    private static final double TARGET_TILT = 0.0;

    private final DrivetrainSubsystem drivetrain;
    private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
    
    private long endTime;
    private boolean wasLevel = false;
    
    /** Creates a new AutoBalanceCommand. */
    public AutoBalanceCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        endTime = System.currentTimeMillis();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double vX = controller.calculate(drivetrain.getPitch(), TARGET_TILT);
        double vY = 0;
        drivetrain.drive(new ChassisSpeeds(vX, vY, 0));

        boolean levelNow = Math.abs(drivetrain.getPitch()) <= TILT_TOLERANCE;
        if (levelNow && !wasLevel) {
            endTime = System.currentTimeMillis() + LEVEL_TIME;
            wasLevel = true;
        } else if (!levelNow) {
            wasLevel = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return wasLevel && System.currentTimeMillis() >= endTime;
    }
}
