package org.troyargonauts.common.util;

public class Gains {
	private double kP;
	private double kI;
	private double kD;
	private double kF;
	private double tolerance;
	private double minOutput;
	private double maxOutput;

	public Gains(double kP, double kI, double kD, double kF, double tolerance) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.tolerance = tolerance;
	}

	public Gains(double kP, double kI, double kD, double tolerance) {
		this(kP, kI, kD, 0, tolerance);
	}

	public Gains(double kP, double kI, double kD) {
		this(kP, kI, kD, 0);
	}

	public Gains(double kP, double kI, double kD, double kF, double minOutput, double maxOutput) {
		this(kP, kI, kD, kF, 0);
		this.maxOutput = maxOutput;
		this.minOutput = minOutput;
	}

	public double getP() {
		return kP;
	}

	public double getI() {
		return kI;
	}

	public double getD() {
		return kD;
	}

	public double getF() {
		return kF;
	}

	public double getMinOutput() {
		return minOutput;
	}

	public double getMaxOutput() {
		return maxOutput;
	}

	public double getTolerance() {
		return tolerance;
	}
}
