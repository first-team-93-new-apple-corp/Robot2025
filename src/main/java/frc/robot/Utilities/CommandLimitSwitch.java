package frc.robot.Utilities;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandLimitSwitch {
    private AnalogInput LimitSwitch;

    public CommandLimitSwitch(int channel) {
        LimitSwitch = new AnalogInput(channel);
    }

    public AnalogInput getInput() {
        return LimitSwitch;
    }

    public boolean triggered(){
        return LimitSwitch.getValue() > 3000;
    }

    public Trigger Tripped() {
        return new Trigger(() -> LimitSwitch.getValue() > 3000);
    }
}
