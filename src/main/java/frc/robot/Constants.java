
package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
    public class Sensors {
        public class DIO {
            public static final int ThroughBoreEncoder = 9;
        }

        public class AnalogIn {
            public static final int HallEffect = 0;
        }

        public class CAN {
            public static final int TOF = 21;
        }

    }

    public class ElevatorConstants {
        public static final int outerElevatorMotorID = 20;
        public static final int innerElevatorMotorID = 21;
        public static final int bottomOuterLimitSwitchID = 22;
        public static final int bottomInnerLimitSwitchID = 23;

        public static final Distance outerL1Setpoint = Inches.of(19);
        public static final Distance outerL2Setpoint = Inches.of(33);
        public static final Distance outerL3Setpoint = Inches.of(49);
        public static final Distance outerL4Setpoint = Inches.of(40);
        public static final Distance innerL1Setpoint = Inches.of(19);
        public static final Distance innerL2Setpoint = Inches.of(33);
        public static final Distance innerL3Setpoint = Inches.of(49);
        public static final Distance innerL4Setpoint = Inches.of(40);

        public static final Distance wheelRadius = Inches.of(1);
        public static final double OuterRotationsToInches = 1;
        public static final double InnerRotationsToInches = 1;

        // public static final double kElevatorGearing = 1;
        // public static final double kCarriageMass = 50;
        // public static final double kElevatorDrumRadius = 1;
        // public static final double kMinElevatorHeightMeters = 0;
        // public static final double kMaxElevatorHeightMeters =
        // Units.Meter.convertFrom(32.9, Inches);
    }

    public class CTRE {
        public static final int Intake = 14;

        public static final int Shoot = 15;

        public static final int Elevator = 16;

        public static final int L_Shoulder = 17;
        public static final int R_Shoulder = 18;

        public static final int FLDrive = 1;
        public static final int FRDrive = 2;
        public static final int BR_Drive = 3;
        public static final int BL_Drive = 4;

        public static final int FL_Steer = 5;
        public static final int FR_Steer = 6;
        public static final int BR_Steer = 7;
        public static final int BL_Steer = 8;

        public static final int FL_Cancoder = 10;
        public static final int FR_Cancoder = 11;
        public static final int BR_Cancoder = 12;
        public static final int BL_CanCoder = 13;

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
