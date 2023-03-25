
package frc.robot.subsystems.arm;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utility.GameMode;
import frc.robot.utility.GameMode.GamePiece;

/** Add your docs here. */
public class ArmPoseLibrary {
    private static HashMap<ArmPoseID, Pose2d> cubePoses = new HashMap<ArmPoseID, Pose2d>();
    private static HashMap<ArmPoseID, Pose2d> conePoses = new HashMap<ArmPoseID, Pose2d>();
    private static HashMap<ArmPoseID, Pose2d> genericPoses = new HashMap<ArmPoseID, Pose2d>();

    public static enum ArmPoseID {
        DOUBLE_STATION_PICKUP,
        SINGLE_STATION_PICKUP,
        FLOOR_PICKUP,
        SCORE_LOW,
        SCORE_MED,
        SCORE_HI,
        INTERMEDIATE,
        STOWED,
        RELEASE;
    }

    static {
        // Poses for cube handling
        cubePoses.put(ArmPoseID.FLOOR_PICKUP, new Pose2d(0.178, 0.082, Rotation2d.fromDegrees(19.0)));
        cubePoses.put(ArmPoseID.SINGLE_STATION_PICKUP, new Pose2d(-0.1, 0.49, Rotation2d.fromDegrees(121.0)));
        cubePoses.put(ArmPoseID.DOUBLE_STATION_PICKUP, new Pose2d(0.136, 0.913, Rotation2d.fromDegrees(57.2)));
        cubePoses.put(ArmPoseID.SCORE_LOW, new Pose2d(-0.01, 0.108, Rotation2d.fromDegrees(65.0)));
        cubePoses.put(ArmPoseID.SCORE_MED, new Pose2d(0.375, 0.540, Rotation2d.fromDegrees(61.000)));
        cubePoses.put(ArmPoseID.SCORE_HI, new Pose2d(0.631, 0.936, Rotation2d.fromDegrees(72.0)));

        // Poses for cone handling
        conePoses.put(ArmPoseID.FLOOR_PICKUP, new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0.0)));
        conePoses.put(ArmPoseID.SINGLE_STATION_PICKUP, new Pose2d(0.041, 0.149, Rotation2d.fromDegrees(54.6)));
        conePoses.put(ArmPoseID.DOUBLE_STATION_PICKUP, new Pose2d(0.026, 0.945, Rotation2d.fromDegrees(30.0)));
        conePoses.put(ArmPoseID.SCORE_LOW, new Pose2d(-0.016, 0.084, Rotation2d.fromDegrees(45.0)));
        conePoses.put(ArmPoseID.SCORE_MED, new Pose2d(0.226, 0.920, Rotation2d.fromDegrees(8.0)));
        conePoses.put(ArmPoseID.SCORE_HI, new Pose2d(0.547,1.076 , Rotation2d.fromDegrees(36.0)));

        // Generic poses that are the same for either cubes or cones
        genericPoses.put(ArmPoseID.STOWED, new Pose2d(-0.016, 0.084, Rotation2d.fromDegrees(113.6)));
        genericPoses.put(ArmPoseID.RELEASE, new Pose2d(-0.28, 0.87, Rotation2d.fromDegrees(105.0)));
        genericPoses.put(ArmPoseID.INTERMEDIATE, new Pose2d(0.1, 1.0, Rotation2d.fromDegrees(105.0)));
    }

    public static Pose2d get(ArmPoseID poseID) {
        Pose2d pose;
        if (GameMode.getGamePiece() == GamePiece.CUBE) {
            pose = cubePoses.get(poseID);
        } else if (GameMode.getGamePiece() == GamePiece.CONE) {
            pose = conePoses.get(poseID);
        } else {
            throw new IllegalArgumentException("Invalid Game Piece");
        }

        if (pose == null) {
            pose = genericPoses.get(poseID);
        }

        if (pose == null) {
            throw new IllegalArgumentException("Pose does not exist");
        }

        return pose;
    }
}
