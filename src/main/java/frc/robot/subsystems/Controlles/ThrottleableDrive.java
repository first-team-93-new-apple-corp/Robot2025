package frc.robot.subsystems.Controlles;

public class ThrottleableDrive extends TwoStickDrive{
    public ThrottleableDrive(int LeftPort, int RightPort){
        super(LeftPort, RightPort);
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