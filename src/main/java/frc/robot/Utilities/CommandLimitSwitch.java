package frc.robot.Utilities;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandLimitSwitch {
    private AnalogInput LimitSwitch;
    private int channel;
    public CommandLimitSwitch(int channel) {
        LimitSwitch = new AnalogInput(channel);
        this.channel = channel;
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

    public void publishValue(){
        SmartDashboard.putNumber("LimitSwitch: Analog Channel " + channel, LimitSwitch.getValue());
    }
}
