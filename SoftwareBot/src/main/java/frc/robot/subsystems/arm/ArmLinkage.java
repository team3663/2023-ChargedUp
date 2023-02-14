/*
 * Class the represents one linkage in our robot arm.
 */

package frc.robot.subsystems.arm;

public class ArmLinkage {
    public final double lengthMeters;
    public final double minAngleRad;
    public final double maxAngleRad;

    public ArmLinkage(double lengthMeters, double minAngleRad, double maxAngleRad) {
        this.lengthMeters = lengthMeters;
        this.minAngleRad = minAngleRad;
        this.maxAngleRad = maxAngleRad;
    }
    
}
