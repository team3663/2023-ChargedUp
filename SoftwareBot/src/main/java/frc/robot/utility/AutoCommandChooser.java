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

        private String defaultCommandName;
        private Supplier<Command> defaultCommandCreator;

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
     * Register the "default" auto command, the command that will be run if the 
     * user makes no other choice.
     * 
     * @param name      Name of default command as it appears in the chooser
     * @param creator   Reference to method to create default command
     */
    public void registerDefaultCreator(String name, Supplier<Command> creator) {
        defaultCommandName = name;
        defaultCommandCreator = creator;
    }

    /**
     * Setup our autonomous command chooser in the Shuffleboard.
     * 
     * NOTE: Call after all of your command creators have been registered.
     * 
     * @param tabName       Name of shuffleboard tab the auto chooser will be shown on
     * @param columnIndex   Column number (zero based) where the chooser displays on the tab
     * @param rowIndex      Row number (zero based) where the chooser displays on the tab
     * @param width         Number of columns the chooser spans
     * @param height        Number of rows the chooser spans
     */
    public void setup(String tabName, int columnIndex, int rowIndex, int width, int height ) {
        List<String> keys = new ArrayList<String>(creators.keySet());
        keys.sort((a, b) -> a.compareTo(b));

        // If a default creator was provided add it to the chooser
        if ( defaultCommandCreator != null ) {
            chooser.setDefaultOption(defaultCommandName, defaultCommandCreator);
        }

        // Add the rest of the registered commands.
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
     * Retrieve the currently selected option from the chooser, create the corresponding
     * command and return it to the caller.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        Supplier<Command> creator = chooser.getSelected();
        return creator.get();
    }
}
