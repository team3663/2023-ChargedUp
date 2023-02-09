package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

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
    private Pose2d targetPose = new Pose2d(0.7, 0.7, Rotation2d.fromDegrees(90.0));
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

    private final MechanismLigament2d currentArmLigament = new MechanismLigament2d("CurrentArm", ARM_LENGTH_METERS, 90.0, 10, new Color8Bit(0, 0, 255));
    private final MechanismLigament2d currentForearmLigament = new MechanismLigament2d("CurrentForearm", FOREARM_LENGTH_METERS, 0.0, 10, new Color8Bit(0, 255, 0));
    private final MechanismLigament2d currentIntakeLigament = new MechanismLigament2d("CurrentIntake", HAND_LENGTH_METERS, 0.0, 10, new Color8Bit(255, 0, 0));

    private final MechanismLigament2d targetArmLigament = new MechanismLigament2d("TargetArm", ARM_LENGTH_METERS, 90.0, 10, new Color8Bit(0, 0, 128));
    private final MechanismLigament2d targetForearmLigament = new MechanismLigament2d("TargetForearm", FOREARM_LENGTH_METERS, 0.0, 10, new Color8Bit(0, 128, 0));
    private final MechanismLigament2d targetIntakeLigament = new MechanismLigament2d("TargetIntake", HAND_LENGTH_METERS, 0.0, 10, new Color8Bit(128, 0, 0));

    private final MechanismRoot2d targetPositionRoot;
    private final MechanismLigament2d targetPositionLigament;

    private final PIDController shoulderController = new PIDController(1.0, 0.0, 0.0);
    private final PIDController elbowController = new PIDController(10.0, 1.0, 2.0);
    private final PIDController wristController = new PIDController(1.0, 0.0, 0.0);

    public ArmSubsystem(ArmIO io) {
        this.io = io;
        this.kinematics = new ArmKinematics(armLengthConstants, armAngleConstraints);

        // Create the mechanism object with a 2M x 2M canvas
        this.mechanism = new Mechanism2d(4, 4);
        MechanismRoot2d root = mechanism.getRoot("shoulder", 2, 2);
        root.append(currentArmLigament);
        currentArmLigament.append(currentForearmLigament);
        currentForearmLigament.append(currentIntakeLigament);

        root.append(targetArmLigament);
        targetArmLigament.append(targetForearmLigament);
        targetForearmLigament.append(targetIntakeLigament);

        targetPositionRoot = mechanism.getRoot("target", 0, 0);
        targetPositionLigament = new MechanismLigament2d("TargetPosition", HAND_LENGTH_METERS, 0, 10, new Color8Bit(255, 255, 255));
        targetPositionRoot.append(targetPositionLigament);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm/Inputs", inputs);

        // Convert our target pose (task space) to an arm state object (c-space) and update the IO object with new values.
        ArmState targetState = kinematics.inverse(targetPose);
//        io.setTargetAngles(currentState.shoulderAngleRad, currentState.elbowAngleRad, currentState.wristAngleRad);

        inputs.shoulderAngleRad = targetState.shoulderAngleRad;
        io.setShoulderVoltage(shoulderController.calculate(inputs.shoulderAngleRad, targetState.shoulderAngleRad));
        io.setElbowVoltage(elbowController.calculate(inputs.elbowAngleRad, targetState.elbowAngleRad));
        io.setWristVoltage(wristController.calculate(inputs.wristAngleRad, targetState.wristAngleRad));

        // Copy the target angles into an array we can pass to AdvantageKit to be logged.
        targetAnglesLogged[0] = targetState.shoulderAngleRad;
        targetAnglesLogged[1] = targetState.elbowAngleRad;
        targetAnglesLogged[2] = targetState.wristAngleRad;

        Logger.getInstance().recordOutput("Arm/Pose", targetPose);
        Logger.getInstance().recordOutput("Arm/TargetAngles", targetAnglesLogged);

        currentArmLigament.setAngle(Units.radiansToDegrees(inputs.shoulderAngleRad));
        currentForearmLigament.setAngle(Units.radiansToDegrees(Math.PI - inputs.elbowAngleRad));
        currentIntakeLigament.setAngle(Units.radiansToDegrees(-inputs.wristAngleRad));

        targetPositionRoot.setPosition(-targetPose.getX() + 2, targetPose.getY() + 2);
        targetPositionLigament.setAngle(targetPose.getRotation().getDegrees());

        targetArmLigament.setAngle(Units.radiansToDegrees(targetState.shoulderAngleRad));
        targetForearmLigament.setAngle(Units.radiansToDegrees(Math.PI - targetState.elbowAngleRad));
        targetIntakeLigament.setAngle(Units.radiansToDegrees(-targetState.wristAngleRad));
        Logger.getInstance().recordOutput("Arm/Mechanism", mechanism);
    }

    public void setPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    public Pose2d getPose() {
        return targetPose;
    }
}
