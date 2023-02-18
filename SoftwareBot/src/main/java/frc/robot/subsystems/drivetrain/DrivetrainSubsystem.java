package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.photonvision.IPhotonVision;
import frc.robot.sim.SimModelData;

import org.littletonrobotics.junction.Logger;

public class DrivetrainSubsystem extends SubsystemBase {
    private static final double WHEELBASE_X_METERS = Units.inchesToMeters(28.0);
    private static final double WHEELBASE_Y_METERS = Units.inchesToMeters(28.0);
    private static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = Units.feetToMeters(13.8);

    private static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND /
            Math.hypot(WHEELBASE_X_METERS / 2.0, WHEELBASE_Y_METERS / 2.0);

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveModule[] swerveModules;
    private final SwerveModulePosition[] modulePositions;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator poseEstimator;


    private ChassisSpeeds targetChassisVelocity = new ChassisSpeeds();
    private double[] chassisVelocityLogged = new double[3];

    private final IPhotonVision photonvision;

    public DrivetrainSubsystem(GyroIO gyroIO,
                               SwerveModuleIO frontLeftModuleIO, SwerveModuleIO frontRightModuleIO,
                               SwerveModuleIO backLeftModuleIO, SwerveModuleIO backRightModuleIO,
                               IPhotonVision photonvision) {
        this.gyroIO = gyroIO;

        this.swerveModules = new SwerveModule[]{
                new SwerveModule("FrontLeftModule", frontLeftModuleIO),
                new SwerveModule("FrontRightModule", frontRightModuleIO),
                new SwerveModule("BackLeftModule", backLeftModuleIO),
                new SwerveModule("BackRightModule", backRightModuleIO)
        };

        modulePositions = new SwerveModulePosition[swerveModules.length];

        for (int i = 0; i < swerveModules.length; ++i) {
            modulePositions[i] = new SwerveModulePosition();
        }

        this.kinematics = new SwerveDriveKinematics(
                // Front Left (+x, +y)
                new Translation2d(WHEELBASE_X_METERS / 2.0, WHEELBASE_Y_METERS / 2.0),
                // Front Right (+x, -y)
                new Translation2d(WHEELBASE_X_METERS / 2.0, -WHEELBASE_Y_METERS / 2.0),
                // Back Left (-x, +y)
                new Translation2d(-WHEELBASE_X_METERS / 2.0, WHEELBASE_Y_METERS / 2.0),
                // Back Right (-x, -y)
                new Translation2d(-WHEELBASE_X_METERS / 2.0, -WHEELBASE_Y_METERS / 2.0)
        );

        Rotation2d initialGyroAngle = new Rotation2d();
        this.odometry = new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions);

        // TODO: Add function for multiple SD's based on # of targets sighted
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, initialGyroAngle, modulePositions, new Pose2d(),
            VecBuilder.fill(0.005, 0.005, 0.0005), VecBuilder.fill(0.5, 0.5, 0.5));

        this.photonvision = photonvision;
    }

    @Override
    public void periodic() {
        photonvision.update();
        // Update gyroscope sensor values
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drivetrain/Gyro", gyroInputs);

        for (int i = 0; i < swerveModules.length; ++i) {
            // Update all the sensor values of each module
            swerveModules[i].updateInputs();

            // Read the current module position
            modulePositions[i] = swerveModules[i].getCurrentPosition();
        }

        // Calculate each module's target state based on the target chassis velocity
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(targetChassisVelocity);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND);

        // Set the target state for each module
        SwerveModuleState[] optimizedModuleStates = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; ++i) {
            // Optimize the module state for the current module position
            optimizedModuleStates[i] = SwerveModuleState.optimize(moduleStates[i], modulePositions[i].angle);
            swerveModules[i].setTargetState(optimizedModuleStates[i]);
        }

        // Copy components of chassis speeds into double array that we can send to AdvantageKit.
        chassisVelocityLogged[0] = targetChassisVelocity.vxMetersPerSecond;
        chassisVelocityLogged[1] = targetChassisVelocity.vyMetersPerSecond;
        chassisVelocityLogged[2] = targetChassisVelocity.omegaRadiansPerSecond;

        Logger.getInstance().recordOutput("Drivetrain/DesiredChassisVelocity", chassisVelocityLogged);
        Logger.getInstance().recordOutput("Drivetrain/DesiredModuleStates", moduleStates);
        Logger.getInstance().recordOutput("Drivetrain/OptimizedModuleStates", optimizedModuleStates);

       // Update odometry and log out pose based on odometry alone
       Pose2d odometryPose = odometry.update(new Rotation2d(gyroInputs.yawRadians), modulePositions);
       Logger.getInstance().recordOutput("Drivetrain/OdometryPose", odometryPose);

        // Calculate new pose using estimator
        Pose2d newPose = poseEstimator.update(new Rotation2d(gyroInputs.yawRadians), modulePositions);

        if (photonvision.getRobotPose3d().isPresent()) {
            poseEstimator.addVisionMeasurement(photonvision.getRobotPose3d().get().estimatedPose.toPose2d(), photonvision.getRobotPose3d().get().timestampSeconds);
            Logger.getInstance().recordOutput("Drivetrain/VisionPose", photonvision.getRobotPose3d().get().estimatedPose.toPose2d());
        }
        
        newPose = poseEstimator.getEstimatedPosition();
        Logger.getInstance().recordOutput("Drivetrain/Pose", newPose);

        photonvision.setReferencePose(newPose);

        // Update simulation model data
        SimModelData.GetInstance().updateDrivetrainData(getPose(), targetChassisVelocity);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.targetChassisVelocity = chassisSpeeds;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose (Pose2d newPose) {
        Rotation2d newGyroYaw = new Rotation2d(gyroInputs.yawRadians);

        poseEstimator.resetPosition(newGyroYaw, modulePositions, newPose);
        odometry.resetPosition(newGyroYaw, modulePositions, newPose);
    }

    public double getMaxTranslationalVelocityMetersPerSecond() {
        return MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND;
    }

    public double getMaxAngularVelocityRadPerSec() {
        return MAX_ANGULAR_VELOCITY_RAD_PER_SEC;
    }

    public ChassisSpeeds getTargetChassisSpeeds () {
        return targetChassisVelocity;
    }
        public void setTargetChassisVelocity(ChassisSpeeds targetChassisVelocity) {
        this.targetChassisVelocity = targetChassisVelocity;
    }

    public SwerveDriveKinematics getKinematics () {
        return kinematics;
    }
}
