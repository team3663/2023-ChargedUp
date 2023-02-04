package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SimModelData {

    // The singleton instance of the model data
    private static SimModelData modelData = null;

    // Drivetrain related model data
    public Pose2d driveTrainPose = new Pose2d();
    public ChassisSpeeds driveTrainChassisSpeeds = new ChassisSpeeds();

    /**
     * 
     * @return Singleton instance of model data.
     */
    public static SimModelData GetInstance()
    {
        if (modelData == null ) {
            modelData = new SimModelData();
        }

        return modelData;
    }

    public void updateDrivetrainData(Pose2d pose, ChassisSpeeds chassisSpeeds) {
        driveTrainPose = pose;
        driveTrainChassisSpeeds = chassisSpeeds;
    }
}
