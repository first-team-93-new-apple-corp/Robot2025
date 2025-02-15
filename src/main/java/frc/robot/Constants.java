
package frc.robot;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {
    public class Inputs {
        public class Cameras {
            public record Camera(String CamName, Transform3d camTransform) {
            }

            public static Camera FrontCam = new Camera("FrontCam",
                    new Transform3d(new Translation3d(Inches.of(-0.5), Inches.of(12.5), Inches.of(6.5)),
                            new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(0))));
            public static Camera RearCam = new Camera("RearCam", new Transform3d(new Translation3d(0, 0, 0),
                    new Rotation3d(0, 0, 0)));
        }

        public class DIO {
            public static final int ThroughBoreEncoder = 9;
        }

        public class AnalogIn {
            public static final int HallEffect = 0;
        }
    }

    public class ArmConstants {
        // Motor ID
        public class IDs {
            public static final int Wrist = 17;
            public static final int Encoder = 99;
        }

        public class Setpoints {
            // Limits
            public static final double lowLimit = 0.0;
            public static final double highLimit = 0.0;
            // Setpoints
            public static final double L1 = 0.0; // TODO find angles
            public static final double L2 = 0.0;
            public static final double L3 = L2;
            public static final double L4 = 10.0;
            public static final double Intake = 0.0; // -90 degree from ground
        }
    }

    public class GrabberConstants {
        // Motor ID (Rev)
        public static final int Grabber = 18;
        // Input ID
        public static final int LimitSwitch = 99; // TDOD get dio pin
        // Speed constants
        public static final double intakeSpeed = 0.5;
        public static final double outakeSpeed = 0.5;
    }

    public class ElevatorConstants {
        public static final int outerElevatorMotorID = 16;
        public static final int innerElevatorMotorID = 15;

        public static final int InnerTopChannel = 1;
        public static final int InnerBottomChannel = 2;
        public static final int OuterTopChannel = 3;
        public static final int OuterBottomChannel = 4;

        public static final Distance L1Setpoint = Centimeters.of(46);
        public static final Distance L2Setpoint = Centimeters.of(81);
        public static final Distance L3Setpoint = Centimeters.of(121);
        public static final Distance L4Setpoint = Centimeters.of(183);

        public static final double SprocketRadiusInches = 1.37 / 2;

        public static final Per<DistanceUnit, AngleUnit> OuterRotationsToInches = Inches
                .of(2 * Math.PI * SprocketRadiusInches).div(Rotations.of(12));

        public static final Per<DistanceUnit, AngleUnit> InnerRotationsToInches = Inches
                .of(2 * Math.PI * SprocketRadiusInches).div(Rotations.of(9));
    }

    public class Drivetrain {

        public static final int FL_Drive = 1;
        public static final int FR_Drive = 2;
        public static final int BR_Drive = 3;
        public static final int BL_Drive = 4;

        public static final int FL_Steer = 5;
        public static final int FR_Steer = 6;
        public static final int BR_Steer = 7;
        public static final int BL_Steer = 8;

        public static final int FL_Cancoder = 10;
        public static final int FR_Cancoder = 11;
        public static final int BR_Cancoder = 12;
        public static final int BL_Cancoder = 13;

    }

    public class Thrustmaster {
        public static final int Trigger = 1;
        public static final int Center_Button = 2;
        public static final int Left_Button = 3;
        public static final int Right_Button = 4;

        public class Left_Buttons {
            public static final int Top_Left = 11;
            public static final int Top_Middle = 12;
            public static final int Top_Right = 13;
            public static final int Bottom_Left = 16;
            public static final int Bottom_Middle = 15;
            public static final int Bottom_Right = 14;
        }

        public class Right_Buttons {
            public static final int Top_Left = 7;
            public static final int Top_Middle = 6;
            public static final int Top_Right = 5;
            public static final int Bottom_Left = 8;
            public static final int Bottom_Middle = 9;
            public static final int Bottom_Right = 10;
        }

        public class Axis {
            public static final int y = 1;
            public static final int x = 0;
            public static final int rotate = 2;
            public static final int slider = 3;
        }
    }

    public class F310_D {
        public static final int X = 1;
        public static final int A = 2;
        public static final int B = 3;
        public static final int Y = 4;
        public static final int LeftShoulderButton = 5;
        public static final int RightShoulderButton = 6;
        public static final int LeftTrigger = 7;
        public static final int RightTrigger = 8;
        public static final int Back = 9;
        public static final int Start = 10;
        public static final int LeftStick = 11;
        public static final int RightStick = 12;

        public class Axis {
            public static final int POV_Y = 0;
            public static final int POV_X = 1;
            public static final int Left_Stick_Y = 0;
            public static final int Left_Stick_X = 1;
            public static final int Right_Stick_Y = 3;
            public static final int Right_Stick_X = 2;
        }
    }

    public class F310_X {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LeftShoulderButton = 5;
        public static final int RightShoulderButton = 6;
        public static final int Back = 7;
        public static final int Start = 8;
        public static final int LeftStick = 9;
        public static final int RightStick = 10;

        public class Axis {
            public static final int POV_Y = 1;
            public static final int POV_X = 0;
            public static final int LT = 2;
            public static final int RT = 3;
            public static final int Left_Stick_Y = 1;
            public static final int Left_Stick_X = 0;
            public static final int Right_Stick_Y = 5;
            public static final int Right_Stick_X = 4;
        }
    }

    public class xbox {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LeftShoulderButton = 5;
        public static final int RightShoulderButton = 6;
        public static final int Window = 7;
        public static final int Menu = 8;
        public static final int LeftPaddle = 9;
        public static final int RightPaddle = 10;

        public class Axis {
            public static final int LT = 2;
            public static final int RT = 3;
            public static final int Left_Stick_Y = 1;
            public static final int Left_Stick_X = 0;
            public static final int Right_Stick_Y = 5;
            public static final int Right_Stick_X = 4;
        }
    }

    public class AprilTags {
        public class RedTags {
            public static final int Proccessor = 16;
            public static final int L_Source = 1;
            public static final int R_Source = 2;
            public static final int BlueSide_Climb = 15;
            public static final int RedSide_Climb = 5;
            // Imagine the reef (hexagon) is q a clock. Looking down from the driverstation
            // with 6 O'Clock facing you.
            public static final int Reef_2_OClock = 6;
            public static final int Reef_4_OClock = 11;
            public static final int Reef_6_OClock = 10;
            public static final int Reef_8_OClock = 9;
            public static final int Reef_10_OClock = 8;
            public static final int Reef_12_OClock = 7;
        }

        public class BlueTags {
            public static final int Proccessor = 3;
            public static final int L_Source = 13;
            public static final int R_Source = 12;
            public static final int BlueSide_Climb = 14;
            public static final int RedSide_Climb = 4;
            // Imagine the reef (hexagon) is a clock. Looking down from the driverstation
            // with 6 O'Clock facing you.
            public static final int Reef_2_OClock = 22;
            public static final int Reef_4_OClock = 17;
            public static final int Reef_6_OClock = 18;
            public static final int Reef_8_OClock = 19;
            public static final int Reef_10_OClock = 20;
            public static final int Reef_12_OClock = 21;

        }
    }
}
