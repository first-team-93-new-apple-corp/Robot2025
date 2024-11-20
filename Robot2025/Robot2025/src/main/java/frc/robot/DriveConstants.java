package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final double yOffset = 0;
    public static final double MaxSpeed = 8;
    public static final double MaxAngularRate = 2 * Math.PI;
    public static final double JoystickDeadzone = 0.1;
    public static final double Track_Width = Units.inchesToMeters(35);
    public static final double Wheel_Base = Units.inchesToMeters(35);
    public static Translation2d Center = new Translation2d(yOffset, 0);
}
