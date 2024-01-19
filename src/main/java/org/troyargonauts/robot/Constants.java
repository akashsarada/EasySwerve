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

    // Angular offsets of the modules relative to the chassis in radians
    double FRONT_LEFT_STARTING_OFFSET = 0;
    double FRONT_RIGHT_STARTING_OFFSET = 0;
    double BACK_LEFT_STARTING_OFFSET = Math.PI/2;
    double BACK_RIGHT_STARTING_OFFSET = Math.PI / 2;

    // SPARK MAX CAN IDs
    int FRONT_LEFT_DRIVING_CAN_ID = 1;
    int REAR_LEFT_DRIVING_CAN_ID = 5;
    int FRONT_RIGHT_DRIVING_CAN_ID = 3;
    int REAR_RIGHT_DRIVING_CAN_ID = 7;

    int FRONT_LEFT_TURNING_CAN_ID = 2;
    int REAR_LEFT_TURNING_CAN_ID = 6;
    int FRONT_RIGHT_TURNING_CAN_ID = 4;
    int REAR_RIGHT_TURNING_CAN_ID = 8;

    int PIGEON = 11;
  }

  public interface OIConstants {
    int DRIVER_CONTROLLER_PORT = 0;
    int OPERATOR_CONTROLLER_PORT = 1;
    double DEADBAND = 0.05;
  }

  public interface ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    int DRIVING_MOTOR_PINION_TEETH = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    boolean TURNING_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    double FREE_SPEED_RPM = 5676;
    double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
    double WHEEL_DIAMETER_METERS = 0.0762;
    double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;

    double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters
    double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

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

    double TURNING_P = 0.25;
    double TURNING_I = 0;
    double TURNING_D = 0;
    double TURNING_FF = 0;
    double TURNING_MIN_OUTPUT = -1;
    double TURNING_MAX_OUTPUT = 1;
  }

  public interface AutoConstants {
    double MAX_SPEED_METERS_PER_SECOND = 3;
    double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    //PID values for the Autonomous PID Controllers
    double X_KP = 1;
    double X_KI = 0;
    double X_KD = 0;

    double Y_KP = 1;
    double Y_KI = 0;
    double Y_KD = 0;

    double THETA_KP = 1;
    double THETA_KI = 0;
    double THETA_KD = 0;
    // Constraint for the motion profiled robot angle controller
    TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }
}
