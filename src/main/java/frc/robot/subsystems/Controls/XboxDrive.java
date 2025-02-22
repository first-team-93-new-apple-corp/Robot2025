package frc.robot.subsystems.Controls;

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

    @Override
    public Trigger superStructureL1() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'superStructureL1'");
    }

    @Override
    public Trigger superStructureL2() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'superStructureL2'");
    }

    @Override
    public Trigger superStructureL3() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'superStructureL3'");
    }

    @Override
    public Trigger superStructureL4() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'superStructureL4'");
    }

    @Override
    public Trigger verticalCoralIntake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'superStructureIntake'");
    }

    @Override
    public Trigger outTake() {
        return Xbox.b();
    }

    @Override
    public Trigger removeAlgea() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'removeAlgea'");
    }

    @Override
    public Trigger bellyPanIntake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'bellyPanIntake'");
    }

    @Override
    public Trigger manUpElev() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manUpElev'");
    }

    @Override
    public Trigger manDownElev() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manDownElev'");
    }

    @Override
    public Trigger manUpArm() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manUpArm'");
    }

    @Override
    public Trigger manDownArm() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manDownArm'");
    }

    @Override
    public Trigger Prime() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'Prime'");
    }
}
