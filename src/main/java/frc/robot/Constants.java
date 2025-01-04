package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public final class Constants {
    public class goals {
        public static Pose3d RedSpeaker = new Pose3d(16.15, 0, 2.032, new Rotation3d());
        public static Pose3d BlueSpeaker = new Pose3d(0.15, 0, 2.032, new Rotation3d());
    }
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

    // Formatted: Shoulder Angle, Elevator Pos, ARM_SECTOR
    public enum ARM_SETPOINTS {
        Intake(-20, -0.01, ARM_SECTOR.INTAKE),
        Amp(67, 75, ARM_SECTOR.AMP),
        Shoot(15, 0, ARM_SECTOR.SHOOT);

        private double shoulderPosition, elevatorPosition;
        private ARM_SECTOR desiredArmSector;

        private ARM_SETPOINTS(double shoulderPosition, double elevatorPosition, ARM_SECTOR desiredArmSector) {
            this.shoulderPosition = shoulderPosition;
            this.elevatorPosition = elevatorPosition;
            this.desiredArmSector = desiredArmSector;
        }

        public double getShoulderPosition() {
            return shoulderPosition;
        }

        public double getElevatorPosition() {
            return elevatorPosition;
        }

        public ARM_SECTOR getDesiredArmSector() {
            return desiredArmSector;
        }
    }

    // Events are all the possible "Modes" of the arm
    public enum ARM_EVENTS {
        ALL_TO_SP,
        SHOULDER_TO_SP,
        RETRACT_TELESCOPE,
        EXTEND_TELESCOPE,
        // Used to stop telescope from running into bumper while leaving stow
        SHOULDER_TO_SP_NC,
        END
    }

    public enum ARM_SECTOR {
        INTAKE,
        AMP,
        SHOOT
    }

    // States are all the possible movements of the arm
    // (ex: intake to amp, amp to shoot, amp to stow, etc.)
    public enum ARM_STATES {

        DO_NOTHING(new ARM_EVENTS[] { ARM_EVENTS.END }),

        // To Intake
        SHOOT_TO_INTAKE(new ARM_EVENTS[] { ARM_EVENTS.SHOULDER_TO_SP, ARM_EVENTS.END }),
        AMP_TO_INTAKE(new ARM_EVENTS[] { ARM_EVENTS.RETRACT_TELESCOPE, ARM_EVENTS.SHOULDER_TO_SP, ARM_EVENTS.END }),
        INTAKE_TO_INTAKE(new ARM_EVENTS[] { ARM_EVENTS.ALL_TO_SP, ARM_EVENTS.END }),
        // To speaker
        INTAKE_TO_SHOOT(new ARM_EVENTS[] { ARM_EVENTS.SHOULDER_TO_SP_NC, ARM_EVENTS.END }),
        AMP_TO_SHOOT(new ARM_EVENTS[] { ARM_EVENTS.RETRACT_TELESCOPE, ARM_EVENTS.SHOULDER_TO_SP, ARM_EVENTS.END }),
        SHOOT_TO_SHOOT(new ARM_EVENTS[] { ARM_EVENTS.ALL_TO_SP, ARM_EVENTS.END }),
        // To amp
        INTAKE_TO_AMP(new ARM_EVENTS[] { ARM_EVENTS.SHOULDER_TO_SP_NC, ARM_EVENTS.EXTEND_TELESCOPE, ARM_EVENTS.END }),
        SHOOT_TO_AMP(new ARM_EVENTS[] { ARM_EVENTS.SHOULDER_TO_SP, ARM_EVENTS.EXTEND_TELESCOPE, ARM_EVENTS.END }),
        AMP_TO_AMP(new ARM_EVENTS[] { ARM_EVENTS.ALL_TO_SP, ARM_EVENTS.END });

        public final ARM_EVENTS[] events;

        private ARM_STATES(ARM_EVENTS[] events) {
            this.events = events;
        }

        public ARM_EVENTS[] getEvents() {
            return events;
        }
    }
}
