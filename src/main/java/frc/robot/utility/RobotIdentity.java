package frc.robot.utility;

import edu.wpi.first.wpilibj.RobotBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import java.util.Set;

@RequiredArgsConstructor
public enum RobotIdentity {
    ROBOT_2022("c2022.yaml"),
    ROBOT_2023("c2023.yaml"),
    SIMULATION("simulation.yaml");

    private static final String ROBOT_2022_MAC_ADDRESS = "00-80-2f-21-c5-21";
    private static final String ROBOT_2023_MAC_ADDRESS = "00-80-2f-33-d0-3f";

    @Getter
    private final String configName;

    /**
     * Gets the identity of the robot.
     *
     * @return The detected identity of the robot.
     */
    public static RobotIdentity getIdentity() {
        // When we're running on a real robot we'll base the identity on its MAC address
        if (RobotBase.isReal()) {
            Set<String> macAddresses = MacAddressUtil.getMacAddresses();
            if (macAddresses.contains(ROBOT_2023_MAC_ADDRESS)) {
                return ROBOT_2023;
            } else if (macAddresses.contains(ROBOT_2022_MAC_ADDRESS)) {
                return ROBOT_2022;
            }

            // If we're unable to determine the robot identity, default to the 2023 robot
            return ROBOT_2023;
        }

        // Otherwise we know we're running in a simulation
        return SIMULATION;
    }
}
