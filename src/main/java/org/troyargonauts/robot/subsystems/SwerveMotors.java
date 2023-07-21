package org.troyargonauts.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.motors.MotorCreation;
import org.troyargonauts.common.motors.wrappers.LazyCANSparkMax;
import org.troyargonauts.robot.Constants;

public class SwerveMotors extends SubsystemBase {
    private final LazyCANSparkMax frontLeftDrive;
    private final LazyCANSparkMax frontLeftTurn;
    private final LazyCANSparkMax frontRightDrive;
    private final LazyCANSparkMax frontRightTurn;
    private final LazyCANSparkMax backLeftDrive;
    private final LazyCANSparkMax backLeftTurn;
    private final LazyCANSparkMax backRightDrive;
    private final LazyCANSparkMax backRightTurn;

    private final LazyCANSparkMax[] driveMotors;
    private final LazyCANSparkMax[] turnMotors;
    private final LazyCANSparkMax[] allMotors;

    private double frontLeftAngle;
    private double frontRightAngle;
    private double backLeftAngle;
    private double backRightAngle;

    private double desiredTargetAngle;

    public SwerveMotors() {
        frontLeftDrive = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.FRONT_LEFT_DRIVE);
        frontLeftTurn = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.FRONT_LEFT_TURN);
        frontRightDrive = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.FRONT_RIGHT_DRIVE);
        frontRightTurn = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.FRONT_RIGHT_TURN);
        backLeftDrive = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.BACK_LEFT_DRIVE);
        backLeftTurn = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.BACK_LEFT_TURN);
        backRightDrive = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.BACK_RIGHT_DRIVE);
        backRightTurn = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.BACK_RIGHT_TURN);

        driveMotors = new LazyCANSparkMax[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive};
        turnMotors = new LazyCANSparkMax[]{frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn};
        allMotors = new LazyCANSparkMax[]{frontLeftDrive, frontLeftTurn, frontRightDrive, frontRightTurn, backLeftDrive, backLeftTurn, backRightDrive, backRightTurn};

        for (LazyCANSparkMax motor : allMotors) {
            motor.setInverted(false);
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
        for (LazyCANSparkMax motor : turnMotors) {
            motor.getPIDController().setP(Constants.SwervePID.DEFAULT_TURN_P, Constants.SwervePID.SLOT);
            motor.getPIDController().setI(Constants.SwervePID.DEFAULT_TURN_I, Constants.SwervePID.SLOT);
            motor.getPIDController().setD(Constants.SwervePID.DEFAULT_TURN_D, Constants.SwervePID.SLOT);
            motor.getPIDController().setOutputRange(-1, 1);
            motor.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.SwervePID.DEFAULT_TURN_TOLERANCE, Constants.SwervePID.SLOT);
        }

        desiredTargetAngle = 0;
    }

    @Override
    public void periodic() {
        frontLeftAngle = frontLeftTurn.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        frontRightAngle = frontRightTurn.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        backLeftAngle = backLeftTurn.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        backRightAngle = backRightTurn.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
    }

    public void run() {
        for (LazyCANSparkMax motor : turnMotors) {
            motor.getPIDController().setReference(desiredTargetAngle, CANSparkMax.ControlType.kPosition, Constants.SwervePID.SLOT);
        }
    }

    public void setPower(double power) {
        for (LazyCANSparkMax motor : driveMotors) {
            motor.set(power);
        }
    }

    public void setAngle(double angle) {
        desiredTargetAngle = angle;
    }

    public LazyCANSparkMax[] getDriveMotors() {
        return driveMotors;
    }

    public LazyCANSparkMax[] getTurnMotors() {
        return turnMotors;
    }

    public LazyCANSparkMax[] getAllMotors() {
        return allMotors;
    }
}

