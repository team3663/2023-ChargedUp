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
import frc.robot.subsystems.intake.IntakeIOComp;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
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

    public static IntakeSubsystem createIntake(RobotIdentity identity) {
        switch (identity) {
            case SIMULATION:
                return new IntakeSubsystem(new IntakeIOSim());
            case ROBOT_2022:
                return new IntakeSubsystem(new IntakeIOSim());
            case ROBOT_2023:
                return new IntakeSubsystem(new IntakeIOComp(20));
            default:
                return new IntakeSubsystem(new IntakeIOSim());
        }
    }
}
