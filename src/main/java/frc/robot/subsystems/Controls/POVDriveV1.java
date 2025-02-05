package frc.robot.subsystems.Controls;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class POVDriveV1 implements ControllerSchemeIO {

    private CommandJoystick LeftStick;
    private CommandJoystick RightStick;

  /**
   * An implementation of  {@link #the (ControllerSchemeIO)}
   * <p> Uses left stick to generate center of rotations
   */
    public POVDriveV1(int LeftPort, int RightPort) {
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);
    }

    @Override
    public double InputLeft() {
        if (LeftStick.button(2).getAsBoolean()) {
            return 0;
        } else {
            return -LeftStick.getY();
        }
    }

    @Override
    public double InputUp() {
        if (LeftStick.button(2).getAsBoolean()) {
            return 0;
        } else {
            return -LeftStick.getX();
        }
    }

    @Override
    public double InputTheta() {
        return -RightStick.getX();
    }

    @Override
    public Translation2d POV() {
        if (LeftStick.button(2).getAsBoolean()) {
            return new Translation2d(-LeftStick.getY() ,-LeftStick.getX());
        } else {
            return new Translation2d(0,0);
        }
    }

    @Override
    public Trigger Seed() {
        return LeftStick.button(12);
    }

    @Override
    public Trigger Brake(){
        return RightStick.trigger();
    }

    @Override
    public Trigger robotRel() {
        return LeftStick.trigger();
    }

    @Override
    public Trigger autoAlign() {
        return LeftStick.button(3);
    }


}