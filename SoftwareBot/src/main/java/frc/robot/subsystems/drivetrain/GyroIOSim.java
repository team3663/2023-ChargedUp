package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sim.SimModelData;
import frc.robot.utility.config.GyroConfig;
import lombok.Data;
import lombok.EqualsAndHashCode;

public class GyroIOSim implements GyroIO {

    private Timer timer = new Timer();
    private double currentYawRads = 0;

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

        // Calculate how much the robot would have rotated at the requested rate during the elapsed time
        // and add it to the current yaw to get the new value.       
        double deltaYaw = requestedVelocity * elapsedTime;
        currentYawRads = MathUtil.inputModulus(currentYawRads + deltaYaw, -Math.PI, Math.PI);
        inputs.yawRadians = currentYawRads;
    }

    @Data
    @EqualsAndHashCode(callSuper = true)
    public static class Config extends GyroConfig {
        @Override
        public GyroIO createIO() {
            return new GyroIOSim();
        }
    }
}
