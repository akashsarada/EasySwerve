// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public interface DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    double MAX_SPEED_METERS_PER_SECOND = 4.8;
    double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    double DIRECTION_SLEW_RATE = 1.2; // radians per second
    double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
    double ROTATIONAL_SLEW_RATE = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    double TRACK_WIDTH = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    double WHEEL_BASE = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // SPARK MAX CAN IDs
    int FRONT_LEFT_DRIVING_CAN_ID = 11;
    int REAR_LEFT_DRIVING_CAN_ID = 13;
    int FRONT_RIGHT_DRIVING_CAN_ID = 15;
    int REAR_RIGHT_DRIVING_CAN_ID = 17;

    int FRONT_LEFT_TURNING_CAN_ID = 10;
    int REAR_LEFT_TURNING_CAN_ID = 12;
    int FRONT_RIGHT_TURNING_CAN_ID = 14;
    int REAR_RIGHT_TURNING_CAN_ID = 16;

    boolean GYRO_REVERSED = false;
  }

  public interface OIConstants {
        int DRIVER_CONTROLLER_PORT = 0;
        int OPERATOR_CONTROLLER_PORT = 1;
        double CONTROLLER_DEADBAND = 0.05;
  }

  public interface ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    int DRIVING_MOTOR_PINION_TEETH = 14;

    // Calculations required for driving motor conversion factors and feed forward
    double FREE_SPEED_RPM = 5676;
    double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
    double WHEEL_DIAMETER_METERS = 0.0762;
    double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
        / DRIVING_MOTOR_REDUCTION;

    double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION; // meters
    double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION) / 60.0; // average meters per second

    double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
    double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
    double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

    double DRIVING_P = 0.04;
    double DRIVING_I = 0;
    double DRIVING_D = 0;
    double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    double DRIVING_MIN_OUTPUT = -1;
    double DRIVING_MAX_OUTPUT = 1;
    double TURNING_P = 1;
    double TURNING_I = 0;
    double TURNING_D = 0;
    double TURNING_FF = 0;
    double TURNING_MIN_OUTPUT = -1;
    double TURNING_MAX_OUTPUT = 1;
  }

  public interface AutoConstants {
    double MAX_VELOCITY = 3; //meters per second
    double MAX_ACCELERATION = 3; // meters per (second squared)
    double MAX_ANGULAR_VELOCITY = Math.PI; // radians per second
    double MAX_ANGULAR_ACCELERATION = Math.PI; // radians per (second squared)

    double PX_CONTROLLER = 1;
    double PY_CONTROLLER = 1;
    double P_THETA_CONTROLLER = 1;

    // Constraint for the motion profiled robot angle controller
    TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);
  }
}
