package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.photonvision.IPhotonVision;
import frc.robot.photonvision.PhotonVisionUtil;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.utility.RobotIdentity;

public final class SubsystemFactory {

    public static IPhotonVision createPhotonvision(RobotIdentity identity) {

        IPhotonVision photon;

        switch (identity) {
            case ROBOT_2022:
                photon = new PhotonVisionUtil(
                    new PhotonCamera[] {
                        new PhotonCamera("Left_Camera"),
                        new PhotonCamera("Right_Camera")
                    },
                    new Transform3d[] {
                        new Transform3d(new Pose3d(), Constants.CameraPoses.LEFT_CAMERA_POSE),
                        new Transform3d(new Pose3d(), Constants.CameraPoses.RIGHT_CAMERA_POSE)
                    }
                );

            default:
                photon = new IPhotonVision() {};
        }

        return photon;
    }

    public static DrivetrainSubsystem createDrivetrain(RobotIdentity identity, IPhotonVision photonvision) {
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
                                Constants.BACK_RIGHT_MODULE_STEER_OFFSET),
                        photonvision);
            case SIMULATION:
                return new DrivetrainSubsystem(
                        new GyroIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        photonvision);
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
                        },
                        photonvision);
        }
    }

    public static ArmSubsystem createArm(RobotIdentity identity) {
        switch (identity) {
            case SIMULATION:
                return new ArmSubsystem(new ArmIOSim());
            default:
                return new ArmSubsystem(new ArmIO() {
                });
        }
    }
}
