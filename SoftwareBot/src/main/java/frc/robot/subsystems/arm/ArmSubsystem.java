package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

import org.littletonrobotics.junction.Logger;

public class ArmSubsystem {

    // Distance from center of robot to shoulder joint along X axis.
    private static final double SHOULDER_X_OFFSET = Units.inchesToMeters(12);

    // Lengths of arms three linkages (arm, forearm & hand)
    private static final double ARM_LENGTH_METERS = Units.inchesToMeters(40);
    private static final double FOREARM_LENGTH_METERS = Units.inchesToMeters(35);
    private static final double HAND_LENGTH_METERS = Units.inchesToMeters(15);

    // angle constraints for each joint
    private static final double SHOULDER_MIN_ANGLE_RAD = Units.degreesToRadians(15);
    private static final double SHOULDER_MAX_ANGLE_RAD = Units.degreesToRadians(90);
    private static final double ELBOW_MIN_ANGLE_RAD = Units.degreesToRadians(10);
    private static final double ELBOW_MAX_ANGLE_RAD = Units.degreesToRadians(100);
    private static final double WRIST_MIN_ANGLE_RAD = Units.degreesToRadians(-90);
    private static final double WRIST_MAX_ANGLE_RAD = Units.degreesToRadians(90);

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private Pose2d targetPose;
    private IArmKinematics kinematics;
    private Mechanism2d mechanism;

    private double[] targetAnglesLogged = new double[3];

    /**
     * Contains the lengths of each arm segment in meters
     * [0] is the arm, [1] is the forearm, and [2] is the hand
     */
    private double[] armLengthConstants = {ARM_LENGTH_METERS, FOREARM_LENGTH_METERS, HAND_LENGTH_METERS};
    /**
     * Contains the angle constraints of each joint
     * [0]/[1] are shoulder min/max, [2]/[3] are elbow min/max, [4]/[5] are wrist min/max
     */
    private double[] armAngleConstraints = {
        SHOULDER_MIN_ANGLE_RAD, SHOULDER_MAX_ANGLE_RAD,
        ELBOW_MIN_ANGLE_RAD, ELBOW_MAX_ANGLE_RAD,
        WRIST_MIN_ANGLE_RAD, WRIST_MAX_ANGLE_RAD
    };

    public ArmSubsystem(ArmIO io) {
        this.io = io;
        this.kinematics = new ArmKinematics(armLengthConstants, armAngleConstraints);

        // Create the mechanism object with a 2M x 2M canvas
        this.mechanism = new Mechanism2d(2, 2);
        MechanismRoot2d root = mechanism.getRoot("shoulder", SHOULDER_X_OFFSET, 0);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm/Inputs", inputs);

        // Convert our target pose (task space) to an arm state object (c-space) and update the IO object with new values.
        ArmState currentState = kinematics.inverse(targetPose);
        io.setTargetAngles(currentState.shoulderAngleRad, currentState.elbowAngleRad, currentState.wristAngleRad);

        // Copy the target angles into an array we can pass to AdvantageKit to be logged.
        targetAnglesLogged[0] = currentState.shoulderAngleRad;
        targetAnglesLogged[1] = currentState.elbowAngleRad;
        targetAnglesLogged[2] = currentState.wristAngleRad;

        Logger.getInstance().recordOutput("Arm/Pose", targetPose);
        Logger.getInstance().recordOutput("Arm/TargetAngles", targetAnglesLogged);
        Logger.getInstance().recordOutput("Arm/Mechanism", mechanism);
    }

    public void setPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    public Pose2d getPose() {
        return targetPose;
    }
}
