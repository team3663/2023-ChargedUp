package frc.robot.utility;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommandChooser {
   
        // Autonomous command creation
        private final HashMap<String, Supplier<Command>> creators = new HashMap<String, Supplier<Command>>();
        private SendableChooser<Supplier<Command>> chooser = new SendableChooser<Supplier<Command>>();

     public AutoCommandChooser() {}
        
    /**
     * Register an autonomous command so it appears in the chooser in Shuffleboard
     * 
     * @param name    Name of command as it appears in the chooser
     * @param creator Reference to method that creates the command
     */
    public void registerCreator(String name, Supplier<Command> creator) {
        creators.put(name, creator);
    }

    /**
     * Setup our autonomous command chooser in the Shuffleboard.
     * Should not be called until after all of your command creators have been registered.
     */
    public void setup(String tabName, int columnIndex, int rowIndex, int width, int height ) {
        List<String> keys = new ArrayList<String>(creators.keySet());
        keys.sort((a, b) -> a.compareTo(b));

        for (String key : keys) {
            chooser.addOption(key, creators.get(key));
        }

        Shuffleboard.getTab(tabName)
                .add("Auto Command", chooser)
                .withPosition(columnIndex, rowIndex)
                .withSize(width, height)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    /**
     * The main {@link Robot} class calls this to get the command to run during
     * autonomous.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        Supplier<Command> creator = chooser.getSelected();
        return creator.get();
    }
}
