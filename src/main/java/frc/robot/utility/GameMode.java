// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** Add your docs here. */
public class GameMode {

    public static enum GamePiece {
        CUBE,
        CONE;
    }

    public static enum PickupLocation {
        FLOOR,
        SINGLE_STATION,
        DOUBLE_STATION;
    }

    public static enum ScoringPosition {
        HIGH,
        MIDDLE,
        LOW;
    }

    private static GamePiece gamePiece = GamePiece.CUBE;
    private static ScoringPosition scoringPosition = ScoringPosition.HIGH;
    private static PickupLocation pickupLocation = PickupLocation.FLOOR;

    private static GenericEntry coneEntry;
    private static GenericEntry cubeEntry;
    private static GenericEntry doubleStation;
    private static GenericEntry singleStation;
    private static GenericEntry floorEntry;
    private static GenericEntry scoreHiEntry;
    private static GenericEntry scoreMedEntry;
    private static GenericEntry scoreLowEntry;

    static {
        SetupShuffleboard();
    }

    public static GamePiece getGamePiece() {
        return gamePiece;
    }

    public static void setGamePiece(GamePiece piece) {
        GameMode.gamePiece = piece;
        Logger.getInstance().recordOutput("Robot/GamePiece", piece.toString());

        coneEntry.setBoolean(piece == GamePiece.CONE);
        cubeEntry.setBoolean(piece == GamePiece.CUBE);
    }

    public static PickupLocation getPickupLocation() {
        return pickupLocation;
    }

    public static void setPickupLocation(PickupLocation location) {
        GameMode.pickupLocation = location;
        Logger.getInstance().recordOutput("Robot/PickupLocation", location.toString());

        doubleStation.setBoolean(location == PickupLocation.DOUBLE_STATION);
        singleStation.setBoolean(location == PickupLocation.SINGLE_STATION);
        floorEntry.setBoolean(location == PickupLocation.FLOOR);
    }

    public static ScoringPosition getScoringPosition() {
        return scoringPosition;
    }

    public static void setScoringPosition(ScoringPosition position) {
        GameMode.scoringPosition = position;
        Logger.getInstance().recordOutput("Robot/ScoringPosition", position.toString());

        scoreHiEntry.setBoolean(position == ScoringPosition.HIGH);
        scoreMedEntry.setBoolean(position == ScoringPosition.MIDDLE);
        scoreLowEntry.setBoolean(position == ScoringPosition.LOW);
    }

    private static void SetupShuffleboard()
    {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver");   

        coneEntry = tab.add("Cone", gamePiece == GamePiece.CONE)
        .withPosition(4, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
        .getEntry();

        cubeEntry = tab.add("Cube", gamePiece == GamePiece.CUBE)
        .withPosition(4, 1)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
        .getEntry();


        doubleStation = tab.add("Double Station", pickupLocation == PickupLocation.DOUBLE_STATION)
        .withPosition(5, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
        .getEntry();

        singleStation = tab.add("Single Station", pickupLocation == PickupLocation.SINGLE_STATION)
        .withPosition(5, 1)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
        .getEntry();

         floorEntry = tab.add("Floor", pickupLocation == PickupLocation.FLOOR)
        .withPosition(5, 2)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
        .getEntry();


        scoreHiEntry = tab.add("High", scoringPosition == ScoringPosition.HIGH)
        .withPosition(6, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
        .getEntry();

        scoreMedEntry = tab.add("Middle", scoringPosition == ScoringPosition.MIDDLE)
        .withPosition(6, 1)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
        .getEntry();        
        
        scoreLowEntry = tab.add("Low", scoringPosition == ScoringPosition.LOW)
        .withPosition(6, 2)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FFFFFF"))
        .getEntry();
    }
}
