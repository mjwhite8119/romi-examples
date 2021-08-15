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
        public static final int TOO_CLOSE = 10;
        public static final int FAR_ENOUGH = 200;
        public static final int END_OF_LINE = 10;
        public static final int SETPOINT = 75;
    }
}
