// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class GameMode {

    public static enum GamePiece {
        CUBE,
        CONE;
    }

    public static enum ScoringPosition {
        HIGH,
        MED,
        LOW;
    }

    public static enum PickupLocation {
        FLOOR,
        SINGLE_STATION,
        DOUBLE_STATION;
    }

    private static GamePiece gamePiece = GamePiece.CUBE;
    private static ScoringPosition scoringPosition = ScoringPosition.HIGH;
    private static PickupLocation pickupLocation = PickupLocation.FLOOR;

    public static GamePiece getGamePiece() {
        return gamePiece;
    }

    public static void setGamePiece(GamePiece mode) {
        GameMode.gamePiece = mode;
        Logger.getInstance().recordOutput("Robot/GamePiece", mode.toString());
    }

    public static ScoringPosition getScoringPosition() {
        return scoringPosition;
    }

    public static void setScoringPosition(ScoringPosition position) {
        GameMode.scoringPosition = position;
        Logger.getInstance().recordOutput("Robot/ScoringPosition", position.toString());
    }

    public static PickupLocation getPickupLocation() {
        return pickupLocation;
    }

    public static void setPickupLocation(PickupLocation location) {
        GameMode.pickupLocation = location;
        Logger.getInstance().recordOutput("Robot/PickupLocation", location.toString());
    }
}
