package frc.robot.util;

import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kDriveKinematics;

import com.flarerobotics.lib.container.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.drive.SwerveSubsystem;

/** A class to assist the driver to get to or align to a certain point. */
public class DriverAssist {
	private SwerveSubsystem m_swerve;
	private SwerveDriveKinematics m_kinematics;

	private PIDController m_controllerX;
	private PIDController m_controllerY;
	private PIDController m_controllerA;

	private boolean m_alignX = true, m_alignY = true, m_alignA = true;

	/**
	 * Constructs a new DriverAssist.
	 *
	 * <p>
	 * The PD constants are in the units of meters and degrees.
	 *
	 * @param swerve        The swerve subsystem.
	 * @param kinematics    The kinematics of the swerve drive.
	 * @param translational The translational PID constants for X and Y. The I constant is omitted.
	 * @param angular       The angular PID constants for rotation. The I constant is omitted.
	 */
	public DriverAssist(SwerveSubsystem swerve, SwerveDriveKinematics kinematics, PIDConstants translational,
			PIDConstants angular) {
		m_swerve = swerve;
		m_kinematics = kinematics;
		m_controllerX = new PIDController(translational.kP, 0.0, translational.kD);
		m_controllerY = new PIDController(translational.kP, 0.0, translational.kD);
		m_controllerA = new PIDController(angular.kP, 0.0, angular.kD);
		m_controllerA.enableContinuousInput(-180, 180);

		// Defaults
		m_controllerX.setTolerance(0.03);
		m_controllerY.setTolerance(0.03);
		m_controllerA.setTolerance(2.0);
	}

	/**
	 * Sets the tolerances for the PD controllers.
	 *
	 * @param tolTranslational The translational tolerance in meters for X and Y.
	 * @param tolAngular       The angular tolerance in degrees for rotation.
	 */
	public void setTolerances(double tolTranslational, double tolAngular) {
		m_controllerX.setTolerance(tolTranslational);
		m_controllerY.setTolerance(tolTranslational);
		m_controllerA.setTolerance(tolAngular);
	}

	/**
	 * Updates the driver assist to drive to the given pose, update periodically even if the point
	 * stays the same to update the velocity output.
	 *
	 * @param point       The point to assist to.
	 * @param alignX      Whether to align the X position.
	 * @param alignY      Whether to align the Y position.
	 * @param alignAngle  Whether to align the angle.
	 * @param maxSpeedMps The maximum speed in meters per second to drive at.
	 * @return The "additional" chassis speeds to drive to the point.
	 */
	public ChassisSpeeds updateAssistTo(Pose2d point, boolean alignX, boolean alignY, boolean alignAngle,
			double maxSpeedMps) {
		if (alignX) m_controllerX.setSetpoint(point.getTranslation().getX());
		if (alignY) m_controllerY.setSetpoint(point.getTranslation().getY());
		if (alignAngle) m_controllerA.setSetpoint(point.getRotation().getDegrees());
		m_alignX = alignX;
		m_alignY = alignY;
		m_alignA = alignAngle;

		Pose2d current = m_swerve.getFilteredPose();
		double A_out = Math.toRadians(m_controllerA.calculate(current.getRotation().getDegrees()));

		var speeds = new ChassisSpeeds(alignX ? m_controllerX.calculate(current.getX()) : 0,
				alignY ? m_controllerY.calculate(current.getY()) : 0, alignAngle ? A_out : 0);
		var states = m_kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMps);
		ChassisSpeeds desaturated = kDriveKinematics.toChassisSpeeds(states);

		return desaturated;
	}

	/**
	 * Returns whether the driver assist is at the setpoint.
	 *
	 * @return Whether the driver assist is at the setpoint.
	 */
	public boolean atSetpoint() {
		return (!m_alignX || m_controllerX.atSetpoint()) && (!m_alignY || m_controllerY.atSetpoint())
				&& (!m_alignA || m_controllerA.atSetpoint());
	}
}
