// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

    public interface SwerveCANIDs {
        byte FRONT_LEFT_DRIVE = 0;
        byte FRONT_LEFT_TURN = 1;
        byte FRONT_RIGHT_DRIVE = 2;
        byte FRONT_RIGHT_TURN = 3;
        byte BACK_LEFT_DRIVE = 4;
        byte BACK_LEFT_TURN = 5;
        byte BACK_RIGHT_DRIVE = 6;
        byte BACK_RIGHT_TURN = 7;
    }

    public interface SwervePID {
        int SLOT = 0;
        double DEFAULT_DRIVE_P = 0.1;
        double DEFAULT_DRIVE_I = 0.0;
        double DEFAULT_DRIVE_D = 0.0;

        double DEFAULT_TURN_P = 0.1;
        double DEFAULT_TURN_I = 0.0;
        double DEFAULT_TURN_D = 0.0;
        double DEFAULT_TURN_TOLERANCE = 1;
        double DEFAULT_TURN_MIN_SPEED = 0.05;
    }

    public interface ControllerPorts {
        int DRIVER_PORT = 0;
        int OPERATOR_PORT = 1;

        double DEADBAND = 0.1;
    }
}
