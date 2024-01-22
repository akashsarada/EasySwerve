package org.troyargonauts.common.motors.wrappers;

// Copyright 2019 FRC Team 3476 Code Orange
// Modified by Team 8728 Troy Argonauts

import com.revrobotics.*;
import org.troyargonauts.common.util.Gains;

/**
 * Sends only new commands to the CANSparkMax to reduce CAN usage.
 */
public class LazyCANSparkMax extends CANSparkMax {

	private final static double EPSILON = 1.0e-6;
	private double prevValue = 0;
	private double prevVoltage = 0;


	public LazyCANSparkMax(int deviceId, MotorType type) {
		super(deviceId, type);
		restoreFactoryDefaults();
		enableVoltageCompensation(10);
	}

	@Override
	public void set(double speed) {
		if (Math.abs(speed - prevValue) > EPSILON) {
			super.set(speed);
			prevValue = speed;
		}
	}

	@Override
	public void setVoltage(double outputVolts) {
		if (Math.abs(outputVolts - prevVoltage) > EPSILON) {
			prevVoltage = outputVolts;
			super.setVoltage(outputVolts);
		}
	}

	public double getSetpoint() {
		return prevValue;
	}

	public double getSetVoltage() {
		return prevVoltage;
	}


	public void setPID(Gains gains) {
		getPIDController().setP(gains.getP());
		getPIDController().setI(gains.getI());
		getPIDController().setD(gains.getD());
		getPIDController().setFF(gains.getF());
		getPIDController().setOutputRange(gains.getMinOutput(), gains.getMaxOutput());
	}

	public void setPIDWrapping(boolean enabled, double minInput, double maxInput) {
		getPIDController().setPositionPIDWrappingEnabled(enabled);
		getPIDController().setPositionPIDWrappingMinInput(minInput);
		getPIDController().setPositionPIDWrappingMaxInput(maxInput);
	}
}
