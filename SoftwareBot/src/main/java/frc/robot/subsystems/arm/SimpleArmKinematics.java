/*
 * Kinematics for our robot arm.
 * 
 * Based on information from the CMU lecture notes and Robot Academy video listed below.
 * 
 * https://www.cs.cmu.edu/~motionplanning/lecture/Chap3-Config-Space_howie.pdf
 * 
 * https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
 * 
 */
package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SimpleArmKinematics implements IArmKinematics {

    private final double armLength;
    private final double forearmLength;

    public SimpleArmKinematics(double armLength, double forearmLength) {
        this.armLength = armLength;
        this.forearmLength = forearmLength;
    }

    /**
     * Forward kinematics - Convert from C-space to task space.
     * 
     * @param state - ArmState object containing a set of joint angles (c-space) for the arm.
     * 
     * @return Pose2d object containing the arm pose (task space) that will result from the provided arm configuration.
     */
    @Override
    public Pose2d forward(ArmState state) {
        
        double x = armLength * Math.cos(state.shoulderAngleRad) + forearmLength * Math.cos(state.shoulderAngleRad + state.elbowAngleRad);
        double y = armLength * Math.sin(state.shoulderAngleRad) + forearmLength * Math.sin(state.shoulderAngleRad + state.elbowAngleRad);

        // Determine the hand angle (relative to x axis)
        double handAngle = state.shoulderAngleRad + state.elbowAngleRad + state.wristAngleRad;

        return new Pose2d(x, y, new Rotation2d(handAngle));
    }

    /**
     * Inverse kinematics - Convert from task space to C-space.
     * 
     * @param pose - Desired pose (task space) for the robot arm.
     * 
     * @return ArmState object conatining joint angles (c-space) that will yield the desired pose.
     */
    @Override    
    public ArmState inverse(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double handAngle = pose.getRotation().getRadians();
    
        // Calculate the elbow angle, there are two possible symetrical solutions for the elbow angle, we choose the negative one since
        // it gives us poses using higher shoulder angles, which we prefer.
        double elbowAngle = -Math.acos((x * x + y * y - armLength * armLength - forearmLength * forearmLength) / (2 * armLength * forearmLength));

        // Calculate the shoulder angle based on the elbow angle we just found.
        double shoulderAngle = Math.atan2(y,x) - Math.atan2(forearmLength * Math.sin(elbowAngle), armLength + forearmLength * Math.cos(elbowAngle));

        // Calculate the angle of the forearm, this is the sum of the shoulder and elbow angles.
        double forearmAngle = shoulderAngle + elbowAngle;

        // Now calculate the wrist angle that gives the desired hand angle based on the forearm angle we just calculated
        double wristAngle = handAngle - forearmAngle;
    
        return new ArmState( shoulderAngle, elbowAngle, wristAngle);
      }
}
