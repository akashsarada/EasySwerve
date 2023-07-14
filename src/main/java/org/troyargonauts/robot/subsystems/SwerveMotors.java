package org.troyargonauts.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.util.motorcontrol.LazyCANSparkMax;
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

    private double frontLeftAngle;
    private double frontRightAngle;
    private double backLeftAngle;
    private double backRightAngle;

    private double desiredTargetAngle;

    public SwerveMotors() {
        frontLeftDrive = new LazyCANSparkMax(Constants.SwerveCANIDs.FRONT_LEFT_DRIVE, CANSparkMax.MotorType.kBrushless);
        frontLeftTurn = new LazyCANSparkMax(Constants.SwerveCANIDs.FRONT_LEFT_TURN, CANSparkMax.MotorType.kBrushless);
        frontRightDrive = new LazyCANSparkMax(Constants.SwerveCANIDs.FRONT_RIGHT_DRIVE, CANSparkMax.MotorType.kBrushless);
        frontRightTurn = new LazyCANSparkMax(Constants.SwerveCANIDs.FRONT_RIGHT_TURN, CANSparkMax.MotorType.kBrushless);
        backLeftDrive = new LazyCANSparkMax(Constants.SwerveCANIDs.BACK_LEFT_DRIVE, CANSparkMax.MotorType.kBrushless);
        backLeftTurn = new LazyCANSparkMax(Constants.SwerveCANIDs.BACK_LEFT_TURN, CANSparkMax.MotorType.kBrushless);
        backRightDrive = new LazyCANSparkMax(Constants.SwerveCANIDs.BACK_RIGHT_DRIVE, CANSparkMax.MotorType.kBrushless);
        backRightTurn = new LazyCANSparkMax(Constants.SwerveCANIDs.BACK_RIGHT_TURN, CANSparkMax.MotorType.kBrushless);

        driveMotors = new LazyCANSparkMax[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive};
        turnMotors = new LazyCANSparkMax[]{frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn};

        for (LazyCANSparkMax motor : driveMotors) {
            motor.getPIDController().setP(Constants.SwervePID.DEFAULT_DRIVE_P);
            motor.getPIDController().setI(Constants.SwervePID.DEFAULT_DRIVE_I);
            motor.getPIDController().setD(Constants.SwervePID.DEFAULT_DRIVE_D);
        }

        for (LazyCANSparkMax motor : turnMotors) {
            motor.getPIDController().setP(Constants.SwervePID.DEFAULT_TURN_P);
            motor.getPIDController().setI(Constants.SwervePID.DEFAULT_TURN_I);
            motor.getPIDController().setD(Constants.SwervePID.DEFAULT_TURN_D);
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
            motor.getPIDController().setReference(desiredTargetAngle, CANSparkMax.ControlType.kPosition);
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
}

