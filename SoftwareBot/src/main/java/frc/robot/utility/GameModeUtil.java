// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class GameModeUtil {
    private static GamePiece mode = GamePiece.CUBE;

    public static void set(GamePiece mode) {
        GameModeUtil.mode = mode;
        Logger.getInstance().recordOutput("Robot/selectedGamePiece", mode.toString());
    }

    public static GamePiece get() {
        return mode;
    }
}
