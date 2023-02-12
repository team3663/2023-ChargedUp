package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SimpleArmKinematics implements IArmKinematics {

    private final double l1;
    private final double l2;

    public SimpleArmKinematics(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
    }

    // Forward kinematics - Convert from C-space to task space.
    @Override
    public Pose2d forward(ArmState state) {
        
        double x = l1 * Math.cos(state.shoulderAngleRad) + l2 * Math.cos(state.shoulderAngleRad + state.elbowAngleRad);
        double y = l1 * Math.sin(state.shoulderAngleRad) + l2 * Math.sin(state.shoulderAngleRad + state.elbowAngleRad);

        return new Pose2d(x, y, new Rotation2d(0.0));
    }

    // Inverse kinematics - Convert from task space to C-space.
    @Override    
    public ArmState inverse(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
    
        double q2 = Math.acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        double q1 = Math.asin((l2 * Math.sin(q2)) / (x * x + y * y)) + Math.atan2(x,y);
    
        return new ArmState( q1, q2, 0);
      }
}
