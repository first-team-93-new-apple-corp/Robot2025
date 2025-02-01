package frc.robot.subsystems.Controlles;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxDrive implements ControllerSchemeIO {

    public CommandXboxController Xbox;

    public XboxDrive(int port) {
        Xbox = new CommandXboxController(port);
    }

    @Override
    public double InputLeft() {
        return -Xbox.getLeftY();
    }

    @Override
    public double InputUp() {
        return -Xbox.getLeftX();
    }



    @Override
    public double InputTheta() {
        return -Xbox.getRightX();
    }

    @Override
    public Translation2d POV() {
        switch (Xbox.getHID().getPOV()) {
            case 0:
                return POVs[1];
            case 90:
                return POVs[3];
            case 180:
                return POVs[5];
            case 270:
                return POVs[7];
            default:
                return POVs[0];
        }
    }

    @Override
    public Trigger Seed() {
        return Xbox.leftBumper();
    }

    @Override
    public Trigger Brake() {
        return Xbox.rightTrigger();
    }
    
    @Override
    public Trigger robotRel() {
        return Xbox.a();
    }
    @Override
    public Trigger autoAlign() {
        return Xbox.x();
    }
}
