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
import frc.robot.utility.RobotIdentity;

public final class SubsystemFactory {

    public static IPhotonVision createPhotonvision(RobotIdentity identity) {

        IPhotonVision photon;

        switch (identity) {
            case ROBOT_2022:
                photon = new PhotonVisionUtil(
                    new PhotonCamera[] {
                        new PhotonCamera("ArduCam"),
                    },
                    new Transform3d[] {
                        new Transform3d(new Pose3d(), Constants.CameraPoses.ARDUCAM_POSE),
                    }
                );
                break;

            case ROBOT_2023:
                    photon = new PhotonVisionUtil(
                        new PhotonCamera[] {
                            new PhotonCamera("Left_Camera"),
                            new PhotonCamera("Right_Camera")
                        },
                        new Transform3d[] {
                            new Transform3d(new Pose3d(), Constants.CameraPoses.LEFT_CAMERA_POSE),
                            new Transform3d(new Pose3d(), Constants.CameraPoses.LEFT_CAMERA_POSE)
                        }
                    );
                    break;

            default:
                photon = new IPhotonVision() {};
                break;
        }

        return photon;
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
