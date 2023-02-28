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
        cubePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(0.45, 0.81, Rotation2d.fromDegrees(45.0)));
        cubePoses.put(ArmPoseID.SCORE_FLOOR, new Pose2d(0.4, 0.2, Rotation2d.fromDegrees(0.0)));
        cubePoses.put(ArmPoseID.SCORE_MED, new Pose2d());
        cubePoses.put(ArmPoseID.SCORE_HI, new Pose2d());

        conePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(0.51, 0.81, Rotation2d.fromDegrees(45.0)));
        conePoses.put(ArmPoseID.SCORE_FLOOR, new Pose2d(0.5, 0.3, Rotation2d.fromDegrees(0.0)));
        conePoses.put(ArmPoseID.SCORE_MED, new Pose2d());
        conePoses.put(ArmPoseID.SCORE_HI, new Pose2d());

        genericPoses.put(ArmPoseID.FLOOR_PICKUP, new Pose2d());
        genericPoses.put(ArmPoseID.STOWED, new Pose2d(0.2, 0.2, Rotation2d.fromDegrees(110.0)));
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
