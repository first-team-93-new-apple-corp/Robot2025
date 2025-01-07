package frc.robot.subsystems.Controlles;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Auton.PositionConstants;

public class DriverAssistTwoStick extends TwoStickDrive {
    private Supplier<Pose2d> robotSupplier;
    private Pose2d goal = PositionConstants.Speaker();
    private PIDController left = new PIDController(2.4, 0, 0.1);
    private PIDController up = new PIDController(2.4, 0, 0.1);

    public DriverAssistTwoStick(int leftport, int rightport, Supplier<Pose2d> robotSupplier) {
        super(leftport, rightport);
        this.robotSupplier = robotSupplier;
        left.setSetpoint(goal.getX());
        up.setSetpoint(goal.getY());

    }

    @Override
    public double DriveLeft() {
        if (LeftStick.button(3).getAsBoolean()) {
            return (super.DriveLeft() / 2) + (up.calculate(robotSupplier.get().getY())/2);
        } else {
            return super.DriveLeft();
        }
    }

    @Override
    public double DriveUp() {
        if (LeftStick.button(3).getAsBoolean()) {
            return (super.DriveUp() / 2) + (-left.calculate(robotSupplier.get().getX())/2);
        } else {
            return super.DriveUp();
        }
    }

    @Override
    public double DriveTheta() {

        return super.DriveTheta();

    }

}
