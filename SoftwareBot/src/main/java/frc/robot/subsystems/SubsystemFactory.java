package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.utility.RobotIdentity;

public final class SubsystemFactory {

    public static DrivetrainSubsystem createDrivetrain(RobotIdentity identity) {
        switch (identity) {
            case ROBOT_2022:
                return new DrivetrainSubsystem(
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
            case SIMULATION:
                return new DrivetrainSubsystem(
                        new GyroIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim());
            default:
                return new DrivetrainSubsystem(
                        new GyroIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        });
        }
    }
}
