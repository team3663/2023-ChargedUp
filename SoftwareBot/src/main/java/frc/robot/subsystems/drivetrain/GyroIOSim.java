package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.sim.SimModelData;

public class GyroIOSim implements GyroIO {

    private Timer timer = new Timer();

    public GyroIOSim() {
        timer.start();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {

        // Sim Gyro does not currently support pitch and roll, only yaw.
        inputs.pitchRadians = 0;
        inputs.rollRadians = 0;

        SimModelData simData = SimModelData.GetInstance();

        // Get the requested angular velocity of rotation (radians/second).
        double requestedVelocity = simData.driveTrainChassisSpeeds.omegaRadiansPerSecond;

        // Determine how much time (seconds) has elapsed since our last update and reset the timer for the next one.
        double elapsedTime = timer.get();
        timer.reset();

        // Calculate how much the robot would have rotated at the requested rate during the elapsed time.
        double deltaYaw = requestedVelocity * elapsedTime;

        // Now add the yaw delta to the current rotation value from the pose to get the new
        // yaw value the gyro sim should return.
        inputs.yawRadians = simData.driveTrainPose.getRotation().getRadians() + deltaYaw;
    }
}
