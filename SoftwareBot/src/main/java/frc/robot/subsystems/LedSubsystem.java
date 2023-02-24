package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LedSubsystem extends SubsystemBase {

    // We are targeting a PWM frequency of 500Hz so there is not too much flicker.

    private final double maxPulseWidthMs = 2.0;
    private final double minPulseWidthMs = 0.0;
    private final double centerPulseWidthMs = (maxPulseWidthMs - minPulseWidthMs) / 2;

    private PWM red;
    private PWM green;
    private PWM blue;

    public LedSubsystem(int redChannel, int greenChannel, int blueChannel) {

        red = new PWM(redChannel);
        green = new PWM(greenChannel);
        blue = new PWM(blueChannel);

        red.setBounds(maxPulseWidthMs, 0.0, centerPulseWidthMs, 0.0, minPulseWidthMs);
        green.setBounds(maxPulseWidthMs, 0.0, centerPulseWidthMs, 0.0, minPulseWidthMs);
        blue.setBounds(maxPulseWidthMs, 0.0, centerPulseWidthMs, 0.0, minPulseWidthMs);
    }

    public void setColor(Color8Bit color) {
        red.setRaw(color.red);
        green.setRaw(color.green);
        blue.setRaw(color.blue);
    }  
}
