package org.troyargonauts.robot.subsystems;

import org.troyargonauts.common.motors.MotorCreation;
import org.troyargonauts.common.motors.wrappers.LazyCANSparkMax;
import org.troyargonauts.robot.Constants.DriveConstants;
import org.troyargonauts.robot.Constants.ModuleConstants;

import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private final LazyCANSparkMax driveMotor;
    private final LazyCANSparkMax turningMotor;

    private final PIDController turningPidController;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, boolean absoluteEncoderReversed) {
        driveMotor = MotorCreation.createDefaultSparkMax(driveMotorId);
        turningMotor = MotorCreation.createDefaultSparkMax(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveMotor.getEncoder().setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveMotor.getEncoder().setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningMotor.getEncoder().setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningMotor.getEncoder().setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(ModuleConstants.kAbsoluteEncoderRot2Rad);
        turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setInverted(absoluteEncoderReversed);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurningPosition() {
        return turningMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public double getAbsoluteEncoderRad() {
        return turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}