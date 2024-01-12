package org.troyargonauts.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public interface ModuleConstants {
        double kWheelDiameterMeters = Units.inchesToMeters(3);
        double kDriveMotorGearRatio = 1 / 5.08;
        double kTurningMotorGearRatio = 1 / 5.08;
        double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        double kAbsoluteEncoderRot2Rad = 2 * Math.PI;
        double kPTurning = 0.0021;
    }

    public interface DriveConstants {
        double kTrackWidth = Units.inchesToMeters(24.5);
        double kWheelBase = Units.inchesToMeters(24.5);

        SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
        );

        int kFrontLeftDriveMotorPort = 1;
        int kBackLeftDriveMotorPort = 5;
        int kFrontRightDriveMotorPort = 3;
        int kBackRightDriveMotorPort = 7;

        int kFrontLeftTurningMotorPort = 2;
        int kBackLeftTurningMotorPort = 6;
        int kFrontRightTurningMotorPort = 4;
        int kBackRightTurningMotorPort = 8;

        int kPigeonPort = 11;

        boolean kFrontLeftTurningEncoderReversed = true;
        boolean kBackLeftTurningEncoderReversed = true;
        boolean kFrontRightTurningEncoderReversed = true;
        boolean kBackRightTurningEncoderReversed = true;

        boolean kFrontLeftDriveEncoderReversed = true;
        boolean kBackLeftDriveEncoderReversed = true;
        boolean kFrontRightDriveEncoderReversed = false;
        boolean kBackRightDriveEncoderReversed = false;

        boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        boolean kBackRightDriveAbsoluteEncoderReversed = false;

        double kPhysicalMaxSpeedMetersPerSecond = 4;
        double kPhysicalMaxAngularSpeedRadiansPerSecond = 10;

        double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public interface ControllerConstants {
        int kDriverControllerPort = 0;

        double kDeadband = 0.05;
    }
}