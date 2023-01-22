package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Timer;

public class GyroIOSim implements GyroIO {

    private DrivetrainSubsystem driveTrain = null;
    private double maxAngularVelocityRadsPerSec = 0.0;
    private Timer timer = new Timer();

    public GyroIOSim() {
    }

    public void setDriveTrain(DrivetrainSubsystem driveTrain){
        this.driveTrain = driveTrain;
        maxAngularVelocityRadsPerSec = driveTrain.getMaxAngularVelocityRadPerSec();
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

            // Sim Gyro does not currently support pitch and roll.
            inputs.pitchRadians = 0;
            inputs.rollRadians = 0;


        }
    }
}
