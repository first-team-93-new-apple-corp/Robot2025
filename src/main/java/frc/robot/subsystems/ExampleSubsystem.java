package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

    public ExampleCommands Commands = new ExampleCommands();
    
    public ExampleSubsystem() {
        // initalize the subsystem
    }

    private void ExampleMethod(double ExampleNumber) {
        //do things
    }

    public class ExampleCommands {
        public Command Example() {
            return runOnce(() -> {
                ExampleMethod(2);
            });
        }
    }
}