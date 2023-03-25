// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmPoseLibrary;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmPoseLibrary.ArmPoseID;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.utility.GameMode;
import frc.robot.utility.GameMode.GamePiece;

public class PlaceCommand extends CommandBase {
    
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;

    /** Creates a new ScoreCommand. */
    public PlaceCommand(ArmSubsystem arm, IntakeSubsystem intake) {
        this.arm = arm;
        this.intake = intake;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        switch (GameMode.getScoringPosition()) {
            case MIDDLE:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.SCORE_MED));
                break;
            case HIGH:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.SCORE_HI));
                break;
            default:
                arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.SCORE_LOW));
                break;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (arm.atTargetPose()) {
            if (GameMode.getGamePiece() == GamePiece.CONE) {
                intake.setPower(-1);
            } else {
                intake.setPower(-0.5);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.setTargetPose(ArmPoseLibrary.get(ArmPoseID.STOWED));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
