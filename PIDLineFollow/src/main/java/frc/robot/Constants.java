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
        // Button mapping for a PS3 gamepad. 
        public static final int SELECT = 1;
        public static final int A = 2;
        public static final int B = 3;
        public static final int START = 4;
        public static final int TOP_DIR = 5;
        public static final int RIGHT_DIR = 6;
        public static final int BOTTOM_DIR = 7;
        public static final int LEFT_DIR = 8;
        public static final int L2_BUTTON = 9;
        public static final int R2_BUTTON = 10;
        public static final int LEFT_ANALOG = 11;
        public static final int RIGHT_ANALOG = 12;
        public static final int TRIANGLE_BUTTON = 13;
        public static final int CIRCLE_BUTTON = 14;
        public static final int CROSS_BUTTON = 15;
        public static final int SQUARE_BUTTON = 16;
        public static final int UNKNOWN_BUTTON = 17;
    }

    public static final class Drive {
        public static final int LEFT_MOTOR_PORT = 7;
        public static final int RIGHT_MOTOR_PORT = 6;

        public static final int[] LEFT_ENCODER_PORTS = new int[] { 0, 1 };
        public static final int[] RIGHT_ENCODER_PORTS = new int[] { 2, 3 };
        public static final boolean LEFT_ENCODER_REVERSED = true;
        public static final boolean RIGHT_ENCODER_REVERSED = false;

        public static final int ENCODER_CPR = 140;
        public static final double WHEEL_DIAMETER_METERS = 0.09;
        public static final double GEAR_RATIO = 72.0 / 90.0;
        public static final double ENCODER_DISTANCE_PER_PULSE = (WHEEL_DIAMETER_METERS * Math.PI)
                / ((double) ENCODER_CPR) * GEAR_RATIO;

        public static final double P = -0.003;
        public static final double KP = -0.015;

    }

    public static final class Vision {
        public static final int END_OF_LINE = 20;
        public static final int SETPOINT = 75;
    }
}
