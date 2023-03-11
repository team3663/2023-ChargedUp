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
        SCORE_HI,
        STOWED;
    }

    static {
        // Low Substation
        cubePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(-0.1, 0.49, Rotation2d.fromDegrees(121.0)));
        // High substation
        // cubePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(0.181, 0.860, Rotation2d.fromDegrees(58.3)));
        cubePoses.put(ArmPoseID.SCORE_FLOOR, new Pose2d(-0.01, 0.108, Rotation2d.fromDegrees(65.0)));
        cubePoses.put(ArmPoseID.SCORE_MED, new Pose2d(-0.1, 0.49, Rotation2d.fromDegrees(121.0)));
        cubePoses.put(ArmPoseID.SCORE_HI, new Pose2d(0.107, 0.846, Rotation2d.fromDegrees(120.0)));

        // Low substation
        conePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(-0.01, 0.108, Rotation2d.fromDegrees(90.0)));
        // High substation
        // conePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(0.224, 1.306, Rotation2d.fromDegrees(-26.7)));
        conePoses.put(ArmPoseID.SCORE_FLOOR, new Pose2d(-0.01, 0.108, Rotation2d.fromDegrees(85.0)));
        conePoses.put(ArmPoseID.SCORE_MED, new Pose2d(0.130, 1.063, Rotation2d.fromDegrees(-18)));
        // Good
        conePoses.put(ArmPoseID.SCORE_HI, new Pose2d(0.247, 1.376, Rotation2d.fromDegrees(0)));
        // Bad
        // conePoses.put(ArmPoseID.SCORE_HI, new Pose2d(0.395, 1.286, Rotation2d.fromDegrees(0)));
        // Worse
        // conePoses.put(ArmPoseID.SCORE_HI, new Pose2d(0.079, 1.274, Rotation2d.fromDegrees(13)));

        genericPoses.put(ArmPoseID.STOWED, new Pose2d(-0.01, 0.108, Rotation2d.fromDegrees(114.0)));
        genericPoses.put(ArmPoseID.FLOOR_PICKUP, new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0.0)));
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
