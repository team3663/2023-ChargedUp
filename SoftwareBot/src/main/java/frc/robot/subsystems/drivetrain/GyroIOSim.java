package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;

public class GyroIOSim implements GyroIO {

    private DrivetrainSubsystem driveTrain = null;
    private DoubleSupplier angularVelocitySupplier = null;
    private Timer timer = new Timer();

    public GyroIOSim() {
    }

    public void initializeModel(DrivetrainSubsystem driveTrain, DoubleSupplier angularVelocitySupplier) {

        this.driveTrain = driveTrain;
        this.angularVelocitySupplier = angularVelocitySupplier;
        timer.start();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {

        if ( driveTrain == null)
        {
            inputs.pitchRadians = 0;
            inputs.rollRadians = 0;
            inputs.yawRadians = 0;
        } else {

            // Sim Gyro does not currently support pitch and roll, only yaw.
            inputs.pitchRadians = 0;
            inputs.rollRadians = 0;

            // Get the requested angular velocity of rotation (radians/second).
            double requestedVelocity = angularVelocitySupplier.getAsDouble();

            // Determine how much time (seconds) has elapsed since our last update and reset the timer for the next one.
            double elapsedTime = timer.get();
            timer.reset();

            // Calculate how much the robot would have rotated at the requested rate during the elapsed time.
            double deltaYaw = requestedVelocity * elapsedTime;

            // Now add the yaw delta to the current rotation value from the pose to get the new
            // yaw value the gyro sim should return.
            Double currentYaw = driveTrain.getPose().getRotation().getRadians();
            inputs.yawRadians = currentYaw + deltaYaw;
        }
    }
}
