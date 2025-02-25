package frc.robot.subsystems.Controls;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TwoStickDriveXboxOp implements ControllerSchemeIO {

    public CommandJoystick LeftStick;
    public CommandJoystick RightStick;
    public CommandXboxController operatorController;

    public TwoStickDriveXboxOp(int LeftPort, int RightPort, int opPort) {
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);
        operatorController = new CommandXboxController(opPort);
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
    public Trigger Brake() {
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

    @Override
    public Trigger superStructureL1() {
        return operatorController.a();
    }

    @Override
    public Trigger superStructureL2() {
        return operatorController.x();
    }

    @Override
    public Trigger superStructureL3() {
        return operatorController.b();
    }

    @Override
    public Trigger superStructureL4() {
        return operatorController.y();
    }

    @Override
    public Trigger verticalCoralIntake() {
        return operatorController.leftTrigger();
    }

    @Override
    public Trigger outTake() {
        return operatorController.rightTrigger();
    }

    @Override
    public Trigger removeAlgea() {
        return operatorController.start();
    }

    @Override
    public Trigger bellyPanIntake() {
        return operatorController.back();
    }

    @Override
    public Trigger manUpElev() {
        return operatorController.povUp();
    }

    @Override
    public Trigger manDownElev() {
        return operatorController.povDown();
    }

    @Override
    public Trigger manUpArm() {
        return operatorController.povRight();
    }

    @Override
    public Trigger manDownArm() {
        return operatorController.povLeft();
    }

    @Override
    public Trigger Prime(){
        return operatorController.rightBumper();
    }

    @Override
    public Trigger climberIn() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'climberIn'");
    }

    @Override
    public Trigger climberOut() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'climberOut'");
    }


}
