package frc.robot.subsystems.Controls;

public class ThrottleableDrive extends TwoStickDriveXboxOp{
    public ThrottleableDrive(int LeftPort, int RightPort, int opPort){
        super(LeftPort, RightPort, opPort);
    }
    private double Speedthrottle(){
        return 1-LeftStick.getThrottle();
    }
    @Override
    public double InputLeft() {
        return -LeftStick.getY() * Speedthrottle();
    }

    @Override
    public double InputUp() {
        return -LeftStick.getX() * Speedthrottle();
    }

    @Override
    public double InputTheta() {
        return -RightStick.getX() * Speedthrottle();
    }
}