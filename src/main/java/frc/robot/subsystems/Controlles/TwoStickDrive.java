package frc.robot.subsystems.Controlles;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TwoStickDrive implements ControllerSchemeIO {

    public CommandJoystick LeftStick;
    public CommandJoystick RightStick;


    public TwoStickDrive(int LeftPort, int RightPort) {
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);

    }

    @Override
    public double InputLeft() {
        return -LeftStick.getY();
    }

    @Override
    public double InputUp() {
        return -LeftStick.getX();
    }

    @Override
    public double InputTheta() {
        return -RightStick.getX();
    }


    @Override
    public Translation2d POV() {
        return AngleToPOV(LeftStick.getHID().getPOV());
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
