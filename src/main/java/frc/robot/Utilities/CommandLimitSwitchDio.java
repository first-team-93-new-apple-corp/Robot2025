package frc.robot.Utilities;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandLimitSwitchDio {
    private DigitalInput LimitSwitch;
    private int channel;
    public CommandLimitSwitchDio(int channel) {
        LimitSwitch = new DigitalInput(channel);
        this.channel = channel;
    }

    public DigitalInput getInput() {
        return LimitSwitch;
    }

    public boolean triggered(){
        return LimitSwitch.get();
    }

    public Trigger Tripped() {
        return new Trigger(() -> triggered());
    }

    public void publishValue(){
        SmartDashboard.putBoolean("LimitSwitch: Analog Channel " + channel, LimitSwitch.get());
    }
}
