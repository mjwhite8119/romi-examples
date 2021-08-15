// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class Joystick {
        // Button mapping for a Logitech gamepad. 
        public static final int X = 1;
        public static final int A = 2;
        public static final int B = 3;
        public static final int Y = 4;
        public static final int TOPLEFT = 5;
        public static final int TOPRIGHT = 6;
        public static final int BOTTOMLEFT = 7;
        public static final int BOTTOMRIGHT = 8;
        public static final int BACK = 9;
        public static final int START = 10;
        public static final int LEFT_ANALOG = 11;
        public static final int RIGHT_ANALOG = 12;
    }

    public final class Servo {
        // Port configuration to match physical configuration on
        // Romi board as well as configuration on http://wpilib.local
        public static final int SERVO1_PORT = 4;
        public static final int SERVO2_PORT = 3;

        public static final double MIN_RANGE = 0.222;
        public static final double MAX_RANGE = 0.722;
        
        // Incremental amount of change for each button press
        // or during the periodic check while holding the button down
        public static final double SERVO_INCREMENT = 0.005;
    }
}
