package com.flarerobotics.lib.control.shooter.data;

import com.flarerobotics.lib.container.Vector3;
import edu.wpi.first.math.geometry.Pose3d;

/** Data class for computed shooter states. */
public class ShooterState {
	private final double m_hoodAngleDeg;
	private final double m_yawRobotOrientedDeg;
	private final double m_yawFieldOrientedDeg;
	private final double m_wheelRPM;
	private final boolean m_isPossible;
	private final double m_exitVelocity;

	private final Vector3 m_exitVelocityVec3;
	private final Pose3d m_exitPosition;

	private final ShooterStateStatus m_status;
	private final String m_debugMessage;

	public ShooterState(double hoodAngleDeg, double yawRobotOrientedDeg, double yawFieldOrientedDeg, double wheelRPM,
			boolean isPossible, double exitVelocity, Vector3 exitVelocityVec, Pose3d exitPosition,
			ShooterStateStatus status, String message) {
		m_hoodAngleDeg = hoodAngleDeg;
		m_yawRobotOrientedDeg = yawRobotOrientedDeg;
		m_yawFieldOrientedDeg = yawFieldOrientedDeg;
		m_wheelRPM = wheelRPM;
		m_isPossible = isPossible;
		m_exitVelocity = exitVelocity;

		m_exitVelocityVec3 = exitVelocityVec;
		m_exitPosition = exitPosition;

		m_status = status;
		m_debugMessage = message;
	}

	/**
	 * Returns the hood angle in degrees.
	 *
	 * @return The angle.
	 */
	public double getHoodAngleDeg() { return m_hoodAngleDeg; }

	/**
	 * Returns the robot oriented turret yaw in degrees. Generally useful for aiming the turret to
	 * look at the target.
	 *
	 * @return The yaw.
	 */
	public double getYawRobotOriented() { return m_yawRobotOrientedDeg; }

	/**
	 * Returns the field oriented turret yaw in degrees. Generally useful for rotating the robot to
	 * look at the target.
	 *
	 * @return The yaw.
	 */
	public double getYawFieldOriented() { return m_yawFieldOrientedDeg; }

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
	 * Returns the field-relative exit velocity of the game piece.
	 *
	 * @return The exit velocity.
	 */
	public double getExitVelocity() { return m_exitVelocity; }

	/**
	 * Returns the 3D exit velocity vector of the game piece in m/s.
	 *
	 * @return The 3D exit velocity vector.
	 */
	public Vector3 getExitVelocityVector() { return m_exitVelocityVec3; }

	/**
	 * Returns the exit position of the game piece.
	 *
	 * @return The exit position.
	 */
	public Pose3d getExitPosition() { return m_exitPosition; }

	/**
	 * Returns the status of the shooter state.
	 *
	 * @return The status.
	 */
	public ShooterStateStatus getStatus() { return m_status; }

	/**
	 * Returns the debug message.
	 *
	 * @return The debug message.
	 */
	public String getDebugMessage() { return m_debugMessage; }

	@Override
	public String toString() {
		return String.format("Hood: %.2fdeg | RPM: %.1f | Possible: %b", m_hoodAngleDeg, m_wheelRPM, m_isPossible);
	}

	// Enums //

	/** An enum representing the final verdict of the state. */
	public enum ShooterStateStatus {
		/** Target is reachable. */
		kOk,
		/** The target is unreachable due to physical constraints. */
		kTargetUnreachable,
		/** A numerical error has occured in the calculations. */
		kNumericalFailure,
	}
}
