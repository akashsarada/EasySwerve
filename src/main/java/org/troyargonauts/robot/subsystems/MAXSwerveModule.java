// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import org.troyargonauts.common.motors.MotorCreation;
import org.troyargonauts.common.motors.wrappers.LazyCANSparkMax;
import org.troyargonauts.robot.Constants.*;


public class MAXSwerveModule {
  private final LazyCANSparkMax drivenMotor, turningMotor;
  private final RelativeEncoder drivenEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController drivenPIDController, turningPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    drivenMotor = MotorCreation.createDriveSwerveSparkMax(drivingCANId);
    turningMotor = MotorCreation.createTurnSwerveSparkMax(turningCANId);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivenEncoder = drivenMotor.getEncoder();
    turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
    drivenPIDController = drivenMotor.getPIDController();
    turningPIDController = turningMotor.getPIDController();
    drivenPIDController.setFeedbackDevice(drivenEncoder);
    turningPIDController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivenEncoder.setPositionConversionFactor(ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
    drivenEncoder.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turningEncoder.setInverted(ModuleConstants.TURNING_ENCODER_INVERTED);

    // Enable PID wrap around for the turning motor.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT);
    turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

    // Set the PID gains for the driving motor.
    drivenPIDController.setP(ModuleConstants.DRIVING_P);
    drivenPIDController.setI(ModuleConstants.DRIVING_I);
    drivenPIDController.setD(ModuleConstants.DRIVING_D);
    drivenPIDController.setFF(ModuleConstants.DRIVING_FF);
    drivenPIDController.setOutputRange(ModuleConstants.DRIVING_MIN_OUTPUT, ModuleConstants.DRIVING_MAX_OUTPUT);

    // Set the PID gains for the turning motor.
    turningPIDController.setP(ModuleConstants.TURNING_P);
    turningPIDController.setI(ModuleConstants.TURNING_I);
    turningPIDController.setD(ModuleConstants.TURNING_D);
    turningPIDController.setFF(ModuleConstants.TURNING_FF);
    turningPIDController.setOutputRange(ModuleConstants.TURNING_MIN_OUTPUT, ModuleConstants.TURNING_MAX_OUTPUT);


    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivenEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(drivenEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        drivenEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivenPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivenEncoder.setPosition(0);
  }
}
