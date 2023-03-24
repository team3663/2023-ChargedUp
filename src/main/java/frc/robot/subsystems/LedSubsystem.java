package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

    private final double pwmRateHz = 100.0;
    private final int colorStep = 100;

    private DigitalOutput red;
    private DigitalOutput green;
    private DigitalOutput blue;
    private Color8Bit curentColor;

    private Color8Bit[] colors;
    private int colorIndex;

    public LedSubsystem(int redChannel, int greenChannel, int blueChannel) {

        red = new DigitalOutput(redChannel);
        red.setPWMRate(pwmRateHz);
        red.enablePWM(0);

        green = new DigitalOutput(greenChannel);
        green.enablePWM(0);

        blue = new DigitalOutput(blueChannel);
        blue.enablePWM(0);

        createColores();

    }

    @Override
    public void periodic() {
        if (this.getCurrentCommand() == null) {
            setColor(colors[colorIndex]);
            colorIndex = ++colorIndex % colors.length;
        }

        long[] RGB = { curentColor.red, curentColor.green, curentColor.blue };

        Logger.getInstance().recordOutput("LED/RGBvalues", RGB);
    }

    public void setColor(Color8Bit color) {

        curentColor = color;

        red.updateDutyCycle((double) color.red / 255.0);
        green.updateDutyCycle((double) color.green / 255.0);
        blue.updateDutyCycle((double) color.blue / 255.0);
    }

    private void createColores() {
        List<Color8Bit> colorList = new ArrayList<Color8Bit>();

        for (int r = 0; r < 100; r++)
            colorList.add(new Color8Bit(r * 255 / colorStep, 255, 0));
        for (int g = 100; g > 0; g--)
            colorList.add(new Color8Bit(255, g * 255 / colorStep, 0));
        for (int b = 0; b < 100; b++)
            colorList.add(new Color8Bit(255, 0, b * 255 / colorStep));
        for (int r = 100; r > 0; r--)
            colorList.add(new Color8Bit(r * 255 / colorStep, 0, 255));
        for (int g = 0; g < 100; g++)
            colorList.add(new Color8Bit(0, g * 255 / colorStep, 255));
        for (int b = 100; b > 0; b--)
            colorList.add(new Color8Bit(0, 255, b * 255 / colorStep));
            
        colorList.add(new Color8Bit(0, 255, 0));
        colors = colorList.toArray(new Color8Bit[colorList.size()]);
    }

}
