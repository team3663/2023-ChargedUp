package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.*;

public final class SubsystemFactory {

    public static DrivetrainSubsystem createDrivetrain() {
        DrivetrainSubsystem drivetrain;
        
        if (Robot.isReal()) {
            drivetrain = new DrivetrainSubsystem(
                    new GyroIOPigeon2(Constants.CanIds.DRIVETRAIN_PIGEON_ID),
                    new SwerveModuleIOFalcon500(Constants.CanIds.DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR,
                            Constants.CanIds.DRIVETRAIN_FRONT_LEFT_MODULE_STEER_MOTOR,
                            Constants.CanIds.DRIVETRAIN_FRONT_LEFT_MODULE_STEER_ENCODER,
                            Constants.FRONT_LEFT_MODULE_STEER_OFFSET),
                    new SwerveModuleIOFalcon500(Constants.CanIds.DRIVETRAIN_FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                            Constants.CanIds.DRIVETRAIN_FRONT_RIGHT_MODULE_STEER_MOTOR,
                            Constants.CanIds.DRIVETRAIN_FRONT_RIGHT_MODULE_STEER_ENCODER,
                            Constants.FRONT_RIGHT_MODULE_STEER_OFFSET),
                    new SwerveModuleIOFalcon500(Constants.CanIds.DRIVETRAIN_BACK_LEFT_MODULE_DRIVE_MOTOR,
                            Constants.CanIds.DRIVETRAIN_BACK_LEFT_MODULE_STEER_MOTOR,
                            Constants.CanIds.DRIVETRAIN_BACK_LEFT_MODULE_STEER_ENCODER,
                            Constants.BACK_LEFT_MODULE_STEER_OFFSET),
                    new SwerveModuleIOFalcon500(Constants.CanIds.DRIVETRAIN_BACK_RIGHT_MODULE_DRIVE_MOTOR,
                            Constants.CanIds.DRIVETRAIN_BACK_RIGHT_MODULE_STEER_MOTOR,
                            Constants.CanIds.DRIVETRAIN_BACK_RIGHT_MODULE_STEER_ENCODER,
                            Constants.BACK_RIGHT_MODULE_STEER_OFFSET));
        } else {

            GyroIOSim gyro = new GyroIOSim();
            
            drivetrain = new DrivetrainSubsystem(gyro,
                    new SwerveModuleIOSim(),
                    new SwerveModuleIOSim(),
                    new SwerveModuleIOSim(),
                    new SwerveModuleIOSim());
        }

        return drivetrain;
    }    
}
