// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utility.GameModeUtil;
import frc.robot.utility.GamePiece;

/** Add your docs here. */
public class ArmPoseLibrary {
    private static HashMap<ArmPoseID, Pose2d> cubePoses = new HashMap<ArmPoseID, Pose2d>();
    private static HashMap<ArmPoseID, Pose2d> conePoses = new HashMap<ArmPoseID, Pose2d>();
    private static HashMap<ArmPoseID, Pose2d> genericPoses = new HashMap<ArmPoseID, Pose2d>();

    public static enum ArmPoseID {
        SUBSTATION_PICKUP,
        FLOOR_PICKUP,
        SCORE_FLOOR,
        SCORE_MED,
        HI_INTERMEDIATE,
        SCORE_HI,
        STOWED;
    }

    static {
        cubePoses.put(ArmPoseID.FLOOR_PICKUP, new Pose2d(0.103, 0.132, Rotation2d.fromDegrees(11.0)));
        // Low Substation
        // cubePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(-0.1, 0.49, Rotation2d.fromDegrees(121.0)));
        // High substation
        cubePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(0.136, 0.913, Rotation2d.fromDegrees(57.2)));
        cubePoses.put(ArmPoseID.SCORE_FLOOR, new Pose2d(-0.01, 0.108, Rotation2d.fromDegrees(65.0)));
        cubePoses.put(ArmPoseID.SCORE_MED, new Pose2d(-0.1, 0.49, Rotation2d.fromDegrees(121.0)));
        cubePoses.put(ArmPoseID.HI_INTERMEDIATE, new Pose2d(-0.01, 0.936, Rotation2d.fromDegrees(0)));
        cubePoses.put(ArmPoseID.SCORE_HI, new Pose2d(0.631, 0.936, Rotation2d.fromDegrees(120.0)));

        conePoses.put(ArmPoseID.FLOOR_PICKUP, new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0.0)));
        // Low substation
        // conePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(0.041, 0.149, Rotation2d.fromDegrees(54.6)));
        // High substation
        conePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(0.296, 1.358, Rotation2d.fromDegrees(-47.3)));
        conePoses.put(ArmPoseID.SCORE_FLOOR, new Pose2d(-0.01, 0.108, Rotation2d.fromDegrees(85.0)));
        conePoses.put(ArmPoseID.SCORE_MED, new Pose2d(0.276, 1.170, Rotation2d.fromDegrees(-28)));
        conePoses.put(ArmPoseID.HI_INTERMEDIATE, new Pose2d(-0.01, 1.376, Rotation2d.fromDegrees(0)));
        conePoses.put(ArmPoseID.SCORE_HI, new Pose2d(0.247, 1.376, Rotation2d.fromDegrees(0)));

        genericPoses.put(ArmPoseID.STOWED, new Pose2d(-0.01, 0.108, Rotation2d.fromDegrees(114.0)));
    }

    public static Pose2d get(ArmPoseID poseID) {
        Pose2d pose;
        if (GameModeUtil.get() == GamePiece.CUBE) {
            pose = cubePoses.get(poseID);
        } else if (GameModeUtil.get() == GamePiece.CONE) {
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
