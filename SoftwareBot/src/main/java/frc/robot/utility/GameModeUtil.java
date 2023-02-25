// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Add your docs here. */
enum GameMode {
    CUBE,
    CONE;
}

public class GameModeUtil {
    private static GameMode mode = GameMode.CUBE;

    public static void set(GameMode mode) {
        GameModeUtil.mode = mode;
    }

    public static GameMode get() {
        return GameModeUtil.mode;
    }
}