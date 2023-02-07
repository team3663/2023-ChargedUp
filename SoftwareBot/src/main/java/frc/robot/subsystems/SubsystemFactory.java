package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.*;

public final class SubsystemFactory {

    public static DrivetrainSubsystem createDrivetrain() {
        DrivetrainSubsystem drivetrain;
        
        if (Robot.isReal()) {
                double cx = Units.inchesToMeters(10.5);
                double cy = Units.inchesToMeters(11.75);
                double cz = Units.inchesToMeters(30);
    
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
                            Constants.BACK_RIGHT_MODULE_STEER_OFFSET),
                    new PhotonCamera[] {
                        new PhotonCamera("Left_Camera"),
                        new PhotonCamera("Right_Camera")
                    },
                    new Transform3d[] {
                        new Transform3d(new Pose3d(), new Pose3d(-cx, cy, cz, new Rotation3d(0, 0, 0.5))),
                        new Transform3d(new Pose3d(), new Pose3d(-cx, -cy, cz, new Rotation3d(0, 0, -0.5)))
                    }
                );
            } else {
                GyroIOSim gyro = new GyroIOSim();
                
                drivetrain = new DrivetrainSubsystem(gyro,
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new PhotonCamera[] {new PhotonCamera("null")},
                        new Transform3d[] {new Transform3d(), new Transform3d()}
                    );


        } 

        return drivetrain;          
    }
}
