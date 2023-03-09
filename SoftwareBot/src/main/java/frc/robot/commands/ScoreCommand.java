// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ScoreCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;

    private SetArmPoseCommand armCommand;
    private IntakeFeedCommand intakeCommand;

    private int c;
    private static final int COMMAND_TIMEOUT = 400;

    /** Creates a new ScoreCommand. */
    public ScoreCommand(ArmSubsystem arm, IntakeSubsystem intake, ArmPoseID poseID) {
        this.arm = arm;
        this.intake = intake;

        armCommand = new SetArmPoseCommand(arm, poseID);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armCommand.initialize();
        c = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        armCommand.execute();
        if (armCommand.isFinished()) {
            intakeCommand = new IntakeFeedCommand(intake, -.8);
            intakeCommand.initialize();
            intakeCommand.execute();
        }
        c++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intakeCommand = new IntakeFeedCommand(intake, -.8);
        } else {
            armCommand = new SetArmPoseCommand(arm, ArmPoseID.STOWED);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return c >= COMMAND_TIMEOUT;
    }
}
