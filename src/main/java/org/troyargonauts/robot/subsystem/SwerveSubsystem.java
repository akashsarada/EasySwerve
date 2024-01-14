package org.troyargonauts.robot.subsystem;

import org.troyargonauts.robot.Constants.DriveConstants;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private final org.troyargonauts.robot.subsystem.SwerveModule frontLeft = new org.troyargonauts.robot.subsystem.SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    );

    private final org.troyargonauts.robot.subsystem.SwerveModule frontRight = new org.troyargonauts.robot.subsystem.SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
    );

    private final org.troyargonauts.robot.subsystem.SwerveModule backLeft = new org.troyargonauts.robot.subsystem.SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
    );

    private final org.troyargonauts.robot.subsystem.SwerveModule backRight = new org.troyargonauts.robot.subsystem.SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed
    );

    public SwerveModulePosition[] getPosition() {
        return new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    }

    private final Pigeon2 gyro = new Pigeon2(DriveConstants.kPigeonPort);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            new Rotation2d(0),
            getPosition()
    );

    public SwerveSubsystem() {
        zeroHeading();
    }

    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(
                getRotation2d(),
                new SwerveModulePosition[]{
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                },
                pose
        );
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getPosition());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Front Right Turn Setpoint", frontRight.getState().angle.getRadians());
        SmartDashboard.putNumber("Front Left Turn Setpoint", frontLeft.getState().angle.getRadians());
        SmartDashboard.putNumber("Back Right Turn Setpoint", backRight.getState().angle.getRadians());
        SmartDashboard.putNumber("Back Left Turn Setpoint", backLeft.getState().angle.getRadians());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}