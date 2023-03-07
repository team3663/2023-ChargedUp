package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LedSubsystem extends SubsystemBase {

    private final double pwmRateHz = 100.0;

    private DigitalOutput red;
    private DigitalOutput green;
    private DigitalOutput blue;

    public LedSubsystem(int redChannel, int greenChannel, int blueChannel) {

        red = new DigitalOutput(redChannel);
        red.setPWMRate(pwmRateHz);
        red.enablePWM(0);

        green = new DigitalOutput(greenChannel);
        green.enablePWM(0);

        blue = new DigitalOutput(blueChannel);
        blue.enablePWM(0);
    }

    public void setColor(Color8Bit color) {

        red.updateDutyCycle((double)color.red / 255.0);
        green.updateDutyCycle((double)color.green / 255.0);
        blue.updateDutyCycle((double)color.blue / 255.0);
    } 
}
