package com.flarerobotics.lib.control.shooter.data;

import com.flarerobotics.lib.math.BilinearInterpolator2D;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * A descriptor for a shooter subsystem, containing all the necessary parameters to calculate
 * the angle and RPM of the shooter, given environmental factors.
 */
public class LibShooterDescriptor {

	// Required parameters
	private final double m_pivotHeight;
	private final double m_rollerRadius;
	private final DCMotor m_gearbox;
	private final double m_rollerReduction;
	private final int m_numMotors;

	// Optional parameters
	private double m_maxRPM = 6000;

	private double m_rollerMOI = 0.01;
	private double m_minAngleDeg = -90;
	private double m_maxAngleDeg = 90;
	private double m_flywheelMOI = 0;
	private double m_flywheelToShooterReduction = 1;
	private double m_hopperDelay = 0.1;
	private double m_nominalVoltage = 12.5;
	private double m_barrelLength = 0.4;
	private double m_validityCheckHeightTol = 0.1;

	private Transform2d m_pivotOffset = new Transform2d();

	// Calculation components
	private BilinearInterpolator2D m_angleInterpolator;
	private BilinearInterpolator2D m_rpmInterpolator;
	private InterpolatingDoubleTreeMap m_windupTimes;

	/**
	 * Constructs a new ShooterDescriptor.
	 *
	 * @param builder The parameter Builder instance.
	 */
	private LibShooterDescriptor(Builder builder) {
		m_pivotHeight = builder.m_pivotHeight;
		m_rollerRadius = builder.m_rollerRadius;
		m_gearbox = builder.m_gearbox;
		m_rollerReduction = builder.m_rollerReduction;
		m_numMotors = builder.m_numMotors;

		m_maxRPM = builder.m_maxRPM;

		m_rollerMOI = builder.m_rollerMOI;
		m_minAngleDeg = builder.m_minAngleDeg;
		m_maxAngleDeg = builder.m_maxAngleDeg;
		m_flywheelMOI = builder.m_flywheelMOI;
		m_flywheelToShooterReduction = builder.m_flywheelToShooterReduction;
		m_hopperDelay = builder.m_hopperDelay;
		m_nominalVoltage = builder.m_nominalVoltage;
		m_barrelLength = builder.m_barrelLength;
		m_validityCheckHeightTol = builder.m_validityCheckHeightTol;
		m_angleInterpolator = builder.m_angleInterpolator;
		m_rpmInterpolator = builder.m_rpmInterpolator;
		m_windupTimes = builder.m_windupTimes;

		m_pivotOffset = builder.m_pivotOffset;
	}

	/** A builder class for ShooterDescriptor paramters. */
	public static class Builder {
		// Required parameters
		private final double m_pivotHeight;
		private final double m_rollerRadius;
		private final DCMotor m_gearbox;
		private final double m_rollerReduction;
		private final int m_numMotors;

		// Optional parameters
		private double m_maxRPM = 6000;

		private double m_rollerMOI = 0;
		private double m_minAngleDeg = -90;
		private double m_maxAngleDeg = 90;
		private double m_flywheelMOI = 0;
		private double m_flywheelToShooterReduction = 1;
		private double m_hopperDelay = 0.1;
		private double m_nominalVoltage = 12.5;
		private double m_barrelLength = 0.4;
		private double m_validityCheckHeightTol = 0.1;
		private BilinearInterpolator2D m_angleInterpolator;
		private BilinearInterpolator2D m_rpmInterpolator;
		private InterpolatingDoubleTreeMap m_windupTimes;

		private Transform2d m_pivotOffset = new Transform2d();

		/**
		 * Constructs a new Builder.
		 *
		 * @param pivotHeight     The height of the pivot point relative to the ground in meters.
		 * @param rollerRadius    The radius of the rollers in meters.
		 * @param gearbox         The DC motor gearbox for the driving motors.
		 * @param rollerReduction The reduction ratio of the rollers, where >1 is a reduction.
		 * @param numMotors       The number of motors driving the shooter rollers.
		 */
		public Builder(double pivotHeight, double rollerRadius, DCMotor gearbox, double rollerReduction,
				int numMotors) {
			m_pivotHeight = pivotHeight;
			m_rollerRadius = rollerRadius;
			m_gearbox = gearbox;
			m_rollerReduction = rollerReduction;
			m_numMotors = numMotors;
			// Absolute hardware limit
			m_maxRPM = m_gearbox.freeSpeedRadPerSec * 60 / (2 * Math.PI * rollerReduction);
		}

		/**
		 * Sets the moment of inertia for the rollers.
		 *
		 * @param value The moment of inertia in kg*m^2.
		 * @return The builder instance for chaining.
		 */
		public Builder rollerMOI(double value) {
			m_rollerMOI = value;
			return this;
		}

		/**
		 * Sets the vertical angle range for the shooter, in degrees.
		 *
		 * @param minDeg The minimum angle in degrees.
		 * @param maxDeg The maximum angle in degrees.
		 * @return The builder instance for chaining.
		 */
		public Builder angleRange(double minDeg, double maxDeg) {
			m_minAngleDeg = minDeg;
			m_maxAngleDeg = maxDeg;
			return this;
		}

		/**
		 * Sets the moment of inertia for the flywheel. Do not use if a flywheel is not attached.
		 *
		 * @param moi       The moment of inertia in kg*m^2.
		 * @param reduction The reduction ratio from the flywheel to the shooter.
		 * @return The builder instance for chaining.
		 */
		public Builder flywheel(double moi, double reduction) {
			m_flywheelMOI = moi;
			m_flywheelToShooterReduction = reduction;
			return this;
		}

		/**
		 * Sets the hopper delay, defined as the time it takes the game piece from the hopper to being
		 * launched, not including the spin-up time. Note that you may need to set this to 0 if you
		 * keep updating the calculator until the note fully exits the robot, the same may apply to the
		 * spin-up times.
		 *
		 * @param seconds The delay.
		 * @return The builder instance for chaining.
		 */
		public Builder hopperDelay(double seconds) {
			m_hopperDelay = seconds;
			return this;
		}

		/**
		 * Sets the length of the barrel from the pivot point to the exit point.
		 *
		 * @param length The length in meters.
		 * @return The builder instance for chaining.
		 */
		public Builder barrelLength(double length) {
			m_barrelLength = length;
			return this;
		}

		/**
		 * Sets the height tolerance for hit validity checks. When the difference between these desired
		 * height and the approximate max height are greater than this value, the shot is considered
		 * invalid. Defaults to 0.1m.
		 *
		 * @param tol The tolerance in meters.
		 * @return The builder instance for chaining.
		 */
		public Builder validityCheckHeightTolerance(double tol) {
			m_validityCheckHeightTol = tol;
			return this;
		}

		/**
		 * Adds a fully-empirical dataset to the shooter descriptor.
		 * <p>
		 *
		 * Example arrays:
		 *
		 * <pre>
		 * <code>
		 * // Sorted Arrays
		 * double[] horizontalDistances = { 2.0, 3.0, 4.0, 5.0 }; // meters
		 * double[] heightDiffs = { -0.5, 0.0, +0.5, +1.0 }; // meters, relative to ground (not pivot)
		 * double[][] angleGrid = {
		 *         // -----------------> HeightDiff (-0.5 -> +1.0)
		 *         { 45, 40, 35, 30 }, // at d=2.0
		 *         { 42, 38, 33, 28 }, // at d=3.0
		 *         { 39, 35, 31, 27 }, // at d=4.0
		 *         { 37, 33, 29, 25 }, // at d=5.0
		 * };
		 * double[][] rpmGrid = {
		 *         // -----------------> HeightDiff (-0.5 -> +1.0)
		 *         { 4500, 4000, 3500, 3000 }, // at d=2.0
		 *         { 4200, 3800, 3300, 2800 }, // at d=3.0
		 *         { 3900, 3500, 3100, 2700 }, // at d=4.0
		 *         { 3700, 3300, 2900, 2500 }, // at d=5.0
		 * }
		 *
		 * // Build windup-time map from 0->4500 RPM
		 * InterpolatingDoubleTreeMap windupTimes = new InterpolatingDoubleTreeMap();
		 * // Always include the (0,0) point:
		 * windupTimes.put(0.0, 0.0);
		 *
		 * windupTimes.put(2500.0, 0.02703);
		 * windupTimes.put(2700.0, 0.02919);
		 * windupTimes.put(2800.0, 0.03027);
		 * windupTimes.put(2900.0, 0.03135);
		 * windupTimes.put(3000.0, 0.03244);
		 * windupTimes.put(3100.0, 0.03352);
		 * windupTimes.put(3300.0, 0.03568);
		 * windupTimes.put(3500.0, 0.03784);
		 * windupTimes.put(3700.0, 0.04000);
		 * windupTimes.put(3800.0, 0.04108);
		 * windupTimes.put(3900.0, 0.04217);
		 * windupTimes.put(4000.0, 0.04325);
		 * windupTimes.put(4200.0, 0.04541);
		 * windupTimes.put(4500.0, 0.04865);
		 * </code> </pre>
		 *
		 * <p>
		 * <i><b>Notes:</b></i>
		 * <p>
		 * <i> <b> - The emprical data must be measured from the shooter's pivot point, not the barrel.
		 * </b> </i>
		 * <p>
		 * <i> <b> - The grids must be sorted in ascending order. </b> </i>
		 * <p>
		 * <i> <b> - The wind-up times must have the data point (0, 0). </b> </i>
		 * <p>
		 * <i> <b> - The data should be large enough to avoid extrapolation due to out-of-bounds data,
		 * which may lead to significant inacuracies. </b> </i>
		 * <p>
		 * <i> <b> - The RPM speeds use the roller RPMs. </b> </i>
		 *
		 * @param angleInterpolator The bilinear interpolator for the angles, as described above.
		 * @param rpmInterpolator   The bilinear interpolator for the RPMs, as described above.
		 * @param windupTimes       The time it takes the motors to spin up to the given RPM from 0
		 *                          RPM.
		 * @return The builder instance for chaining.
		 */
		public Builder dataSet(BilinearInterpolator2D angleInterpolator, BilinearInterpolator2D rpmInterpolator,
				InterpolatingDoubleTreeMap windupTimes) {
			m_angleInterpolator = angleInterpolator;
			m_rpmInterpolator = rpmInterpolator;
			m_windupTimes = windupTimes;
			return this;
		}

		/**
		 * Adds a fully-empirical dataset to the shooter descriptor. <b>Assumes the wind-up times are
		 * negligible.</b>
		 * <p>
		 *
		 * Example arrays:
		 *
		 * <pre>
		 * <code>
		 * // Sorted Arrays
		 * double[] horizontalDistances = { 2.0, 3.0, 4.0, 5.0 }; // meters
		 * double[] heightDiffs = { -0.5, 0.0, +0.5, +1.0 }; // meters, relative to ground (not pivot)
		 * double[][] angleGrid = {
		 *         // -----------------> HeightDiff (-0.5 -> +1.0)
		 *         { 45, 40, 35, 30 }, // at d=2.0
		 *         { 42, 38, 33, 28 }, // at d=3.0
		 *         { 39, 35, 31, 27 }, // at d=4.0
		 *         { 37, 33, 29, 25 }, // at d=5.0
		 * };
		 * double[][] rpmGrid = {
		 *         // -----------------> HeightDiff (-0.5 -> +1.0)
		 *         { 4500, 4000, 3500, 3000 }, // at d=2.0
		 *         { 4200, 3800, 3300, 2800 }, // at d=3.0
		 *         { 3900, 3500, 3100, 2700 }, // at d=4.0
		 *         { 3700, 3300, 2900, 2500 }, // at d=5.0
		 * }
		 * </code> </pre>
		 *
		 * <p>
		 * <i><b>Notes:</b></i>
		 * <p>
		 * <i> <b> - The emprical data must be measured from the shooter's pivot point, not the barrel.
		 * </b> </i>
		 * <p>
		 * <i> <b> - The grids must be sorted in ascending order. </b> </i>
		 * <p>
		 * <i> <b> - The wind-up times must have the data point (0, 0). </b> </i>
		 * <p>
		 * <i> <b> - The data should be large enough to avoid extrapolation due to out-of-bounds data,
		 * which may lead to significant inacuracies. </b> </i>
		 * <p>
		 * <i> <b> - The RPM speeds use the roller RPMs. </b> </i>
		 *
		 * @param angleInterpolator The bilinear interpolator for the angles, as described above.
		 * @param rpmInterpolator   The bilinear interpolator for the RPMs, as described above.
		 * @return The builder instance for chaining.
		 */
		public Builder dataSet(BilinearInterpolator2D angleInterpolator, BilinearInterpolator2D rpmInterpolator) {
			m_angleInterpolator = angleInterpolator;
			m_rpmInterpolator = rpmInterpolator;
			return this;
		}

		/**
		 * Sets the birdseye pivot offset relative to the robot's center.
		 *
		 * @param offset The offset.
		 * @return The builder instance for chaining.
		 */
		public Builder pivotOffset(Transform2d offset) {
			m_pivotOffset = offset;
			return this;
		}

		/**
		 * Sets the nominal battery voltage.
		 *
		 * @param value The nominal voltage in Volts, typically 12V - 12.5V.
		 * @return The builder instance for chaining.
		 */
		public Builder nominalVoltage(double value) {
			m_nominalVoltage = value;
			return this;
		}

		/**
		 * Sets the RPM limits for the shooter. Defaults to the value computed via
		 * <code>maxRPM/reduction</code>, the hardware maximum.
		 *
		 * @param maxRPM The maximum RPM.
		 * @return The builder instance for chaining.
		 */
		public Builder setRPMLimits(double maxRPM) {
			m_maxRPM = maxRPM;
			return this;
		}

		/**
		 * Builds the ShooterDescriptor instance with the provided parameters.
		 *
		 * @return A new ShooterDescriptor instance.
		 */
		public LibShooterDescriptor build() {
			return new LibShooterDescriptor(this);
		}
	}

	// Accessors
	public double getPivotHeight() { return m_pivotHeight; }

	public double getRollerRadius() { return m_rollerRadius; }

	public double getRollerMOI() { return m_rollerMOI; }

	public DCMotor getGearbox() { return m_gearbox; }

	public double getRollerReduction() { return m_rollerReduction; }

	public int getNumMotors() { return m_numMotors; }

	public double getMaxRPM() { return m_maxRPM; }

	public double getMinAngle() { return m_minAngleDeg; }

	public double getMaxAngle() { return m_maxAngleDeg; }

	public double getFlywheelMOI() { return m_flywheelMOI; }

	public double getFlywheelToShooterReduction() { return m_flywheelToShooterReduction; }

	/**
	 * Returns the hopper delay, defined as the time it takes the game piece from the hopper to
	 * being launched, not including the spin-up time.
	 *
	 * @return The delay.
	 */
	public double getHopperDelay() { return m_hopperDelay; }

	public double getNominalVoltage() { return m_nominalVoltage; }

	public double getBarrelLength() { return m_barrelLength; }

	public double getValidityCheckHeightTol() { return m_validityCheckHeightTol; }

	public BilinearInterpolator2D getAngleInterpolator() { return m_angleInterpolator; }

	public BilinearInterpolator2D getRPMInterpolator() { return m_rpmInterpolator; }

	public InterpolatingDoubleTreeMap getWindupTimes() { return m_windupTimes; }

	public Transform2d getPivotOffset() { return m_pivotOffset; }
}
