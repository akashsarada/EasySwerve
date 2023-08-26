package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;
import org.troyargonauts.common.math.Vector2D;
import org.troyargonauts.common.motors.MotorCreation;
import org.troyargonauts.common.motors.wrappers.LazyCANSparkMax;
import org.troyargonauts.common.util.imu.Pigeon;
import org.troyargonauts.robot.Constants;
import org.troyargonauts.robot.Main;

public class Swerve extends SubsystemBase {
    private final LazyCANSparkMax frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private final LazyCANSparkMax frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn;
    private final LazyCANSparkMax[] driveMotors, turnMotors, allMotors;
    private final Pigeon2 pigeon;

    private double frontLeftAngle, frontRightAngle, backLeftAngle, backRightAngle, pigeonCompass;
    private double[] targetAngles;

    public Swerve() {
        frontLeftDrive = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.FRONT_LEFT_DRIVE);
        frontLeftTurn = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.FRONT_LEFT_TURN);
        frontRightDrive = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.FRONT_RIGHT_DRIVE);
        frontRightTurn = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.FRONT_RIGHT_TURN);
        backLeftDrive = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.BACK_LEFT_DRIVE);
        backLeftTurn = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.BACK_LEFT_TURN);
        backRightDrive = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.BACK_RIGHT_DRIVE);
        backRightTurn = MotorCreation.createDefaultSparkMax(Constants.SwerveCANIDs.BACK_RIGHT_TURN);

        pigeon = new Pigeon2(Constants.SwerveCANIDs.PIGEON);

        driveMotors = new LazyCANSparkMax[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive};
        turnMotors = new LazyCANSparkMax[]{frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn};
        allMotors = new LazyCANSparkMax[]{frontLeftDrive, frontLeftTurn, frontRightDrive, frontRightTurn, backLeftDrive, backLeftTurn, backRightDrive, backRightTurn};

        targetAngles = new double[]{0,0,0,0};

        for (LazyCANSparkMax motor : allMotors) {
            motor.setInverted(false);
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            motor.set(0);
        }

        for (LazyCANSparkMax motor : turnMotors) {
            motor.getPIDController().setP(Constants.SwervePID.DEFAULT_TURN_P, Constants.SwervePID.SLOT);
            motor.getPIDController().setI(Constants.SwervePID.DEFAULT_TURN_I, Constants.SwervePID.SLOT);
            motor.getPIDController().setD(Constants.SwervePID.DEFAULT_TURN_D, Constants.SwervePID.SLOT);
            motor.getPIDController().setOutputRange(-1, 1);
            motor.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.SwervePID.DEFAULT_TURN_TOLERANCE, Constants.SwervePID.SLOT);
        }

        pigeon.configEnableCompass(true);
        pigeon.setYawToCompass();
    }

    @Override
    public void periodic() {
        frontLeftAngle = frontLeftTurn.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        frontRightAngle = frontRightTurn.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        backLeftAngle = backLeftTurn.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        backRightAngle = backRightTurn.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();

        pigeonCompass = pigeon.getAbsoluteCompassHeading();
    }

    public void run() {
        frontLeftTurn.getPIDController().setReference(targetAngles[0], CANSparkMax.ControlType.kPosition, Constants.SwervePID.SLOT);
        frontRightTurn.getPIDController().setReference(targetAngles[1], CANSparkMax.ControlType.kPosition, Constants.SwervePID.SLOT);
        backLeftTurn.getPIDController().setReference(targetAngles[2], CANSparkMax.ControlType.kPosition, Constants.SwervePID.SLOT);
        backRightTurn.getPIDController().setReference(targetAngles[3], CANSparkMax.ControlType.kPosition, Constants.SwervePID.SLOT);
    }

    private void setPower(double[] powers) {
        frontLeftDrive.set(powers[0]);
        frontRightDrive.set(powers[1]);
        backLeftDrive.set(powers[2]);
        backRightDrive.set(powers[2]);
    }

    private void setAngle(double[] angles) {
        targetAngles[0] = angles[0];
        targetAngles[1] = angles[1];
        targetAngles[2] = angles[2];
        targetAngles[3] = angles[3];
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

    public double fieldCentrifyForward (double robotAngle, double stickY, double stickX){

        return (stickY * Math.cos(pigeonCompass)) + (stickX * Math.sin(pigeonCompass));

    }

    public double fieldCentrifyStrafe (double stickY, double stickX, double robotAngle){

        return (stickY * Math.sin(robotAngle)) + (stickX * Math.cos(robotAngle));

    }


    private double[] calculateCoefficients (double forward, double strafe, double turn) {
        double[] coefficients = new double[4];

        coefficients[0] = strafe - (turn * (Constants.RobotParameters.ROBOT_LENGTH / 2));
        coefficients[1] = strafe + (turn * (Constants.RobotParameters.ROBOT_LENGTH / 2));
        coefficients[2] = forward - (turn * (Constants.RobotParameters.ROBOT_WIDTH / 2));
        coefficients[3] = forward + (turn * (Constants.RobotParameters.ROBOT_WIDTH / 2));

        return coefficients;
    }
    private double[] calculatePower(double[] coefficients) {

        double[] power = new double[4];

        power[0] = Math.sqrt(Math.pow(coefficients[1], 2) + Math.pow(coefficients[2], 2));
        power[1] = Math.sqrt(Math.pow(coefficients[1], 2) + Math.pow(coefficients[3], 2));
        power[2] = Math.sqrt(Math.pow(coefficients[0], 2) + Math.pow(coefficients[3], 2));
        power[3] = Math.sqrt(Math.pow(coefficients[0], 2) + Math.pow(coefficients[2], 2));

        return power;

    }

    private double[] calculateAngle(double[] coefficients) {
        double[] angle = new double[4];

        angle[0] = Math.toDegrees(Math.atan2(coefficients[1], coefficients[2]));
        angle[1] = Math.toDegrees(Math.atan2(coefficients[1], coefficients[3]));
        angle[2] = Math.toDegrees(Math.atan2(coefficients[0], coefficients[3]));
        angle[3] = Math.toDegrees(Math.atan2(coefficients[0], coefficients[2]));

        return angle;
    }

    public void setSwerve(double forward, double strafe, double turn) {
        double[] coefficients = calculateCoefficients(fieldCentrifyForward(forward, strafe, pigeonCompass), fieldCentrifyStrafe(forward, strafe, pigeonCompass), turn);

        setPower(calculatePower(coefficients));
        setAngle(calculateAngle(coefficients));
    }
}

