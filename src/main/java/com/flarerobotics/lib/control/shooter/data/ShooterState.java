package com.flarerobotics.lib.control.shooter.data;

/** Data class for computed shooter states. */
public class ShooterState {
	private final double m_hoodAngleDeg;
	private final double m_yawRobotOrientedDeg;
	private final double m_wheelRPM;
	private final boolean m_isPossible;
	private final double m_exitVelocity;

	public ShooterState(double hoodAngleDeg, double yawRobotOrientedDeg, double wheelRPM, boolean isPossible,
			double exitVelocity) {
		m_hoodAngleDeg = hoodAngleDeg;
		m_yawRobotOrientedDeg = yawRobotOrientedDeg;
		m_wheelRPM = wheelRPM;
		m_isPossible = isPossible;
		m_exitVelocity = exitVelocity;
	}

	/**
	 * Returns the hood angle in degrees.
	 *
	 * @return The angle.
	 */
	public double getHoodAngleDeg() { return m_hoodAngleDeg; }

	/**
	 * Returns the robot oriented turret yaw in degrees.
	 *
	 * @return The yaw.
	 */
	public double getYawRobotOriented() { return m_yawRobotOrientedDeg; }

	/**
	 * Returns the flywheel RPM.
	 *
	 * @return The RPM.
	 */
	public double getWheelRPM() { return m_wheelRPM; }

	/**
	 * Returns whether the target is reachable, assuming a pivoting shooter.
	 *
	 * @return true if reachable, false otherwise.
	 */
	public boolean isPossible() { return m_isPossible; }

	/**
	 * Returns the exit velocity of the game piece.
	 *
	 * @return The exit velocity.
	 */
	public double getExitVelocity() { return m_exitVelocity; }

	@Override
	public String toString() {
		return String.format("Hood: %.2fdeg | RPM: %.1f | Possible: %b", m_hoodAngleDeg, m_wheelRPM, m_isPossible);
	}
}
