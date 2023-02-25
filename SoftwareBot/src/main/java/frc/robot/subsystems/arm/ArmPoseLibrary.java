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
        SCORE_FLOOR,
        STOWED;
    }

    static {
        cubePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(0.45, 0.81, Rotation2d.fromDegrees(0.0)));
        conePoses.put(ArmPoseID.SUBSTATION_PICKUP, new Pose2d(0.51, 0.81, Rotation2d.fromDegrees(0.0)));
        genericPoses.put(ArmPoseID.STOWED, new Pose2d(0.11, 0.16, Rotation2d.fromDegrees(110.0)));
    }

    public static Pose2d get(ArmPoseID poseID) {
        if (GameModeUtil.get() == GamePiece.CUBE) {
            if (cubePoses.containsKey(poseID)) {
                return cubePoses.get(poseID);
            } else {
                return genericPoses.get(poseID);
            }
        } else if (GameModeUtil.get() == GamePiece.CONE) {
            if (conePoses.get(poseID) != null) {
                return conePoses.get(poseID);
            } else {
                return genericPoses.get(poseID);
            }
        } else {
            throw new IllegalArgumentException("There's been an error with the GameModeUtil");
        }
    }
}
