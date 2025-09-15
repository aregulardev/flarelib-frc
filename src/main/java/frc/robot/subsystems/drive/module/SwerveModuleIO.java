package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The main superclass interface for swerve module (hardware) interaction IO classes.
 */
public interface SwerveModuleIO {

	/** The inputs class for the Swerve Modules. */
	@AutoLog
	public class SwerveModuleIOInputs {
		public boolean driveMotorConnected = true;
		public double drivePositionMeters = 0.0;
		public double driveVelocityMetersPerSec = 0.0;
		public double driveAppliedVolts = 0.0;
		public double driveSupplyCurrentAmps = 0.0;
		public double driveStatorCurrentAmps = 0.0;
		public double driveTemperatureCelsius = 0.0;

		public boolean turnMotorConnected = true;
		public Rotation2d turnPosition = new Rotation2d();
		public double turnVelocityRadPerSec = 0.0;
		public double turnAppliedVolts = 0.0;
		public double turnSupplyCurrentAmps = 0.0;
		public double turnStatorCurrentAmps = 0.0;
		public double turnTemperatureCelsius = 0.0;

		public double[] odometryTimestamps = new double[] {};
		public double[] odometryDrivePositionsMeters = new double[] {};
		public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
	}

	/**
	 * Updates the given module inputs.
	 *
	 * @param inputs The inputs.
	 */
	public default void updateInputs(SwerveModuleIOInputs inputs) {}

	/**
	 * Runs the drive motors at the given output.
	 *
	 * @param output The output.
	 */
	public default void runDriveOpenLoop(double output) {}

	/**
	 * Runs the turn motors at the given output.
	 *
	 * @param output The output.
	 */
	public default void runTurnOpenLoop(double output) {}

	/**
	 * Runs the drive motors at the given velocity.
	 *
	 * @param velocityMetersPerSec The velocity.
	 */
	public default void runDriveVelocity(double velocityMetersPerSec) {}

	/**
	 * Runs the drive motors at the given velocity.
	 *
	 * @param velocityMetersPerSec The velocity.
	 * @param ff                   The feedforward voltage.
	 */
	public default void runDriveVelocityFF(double velocityMetersPerSec, double ff) {}

	/**
	 * Runs the drive motors at the given velocity.
	 *
	 * @param velocityMetersPerSec The velocity.
	 * @param ff                   The feedforward voltage.
	 * @param onlyProvidedFF       Whether to only use the provided feedforward voltage or not.
	 */
	public default void runDriveVelocityFF(double velocityMetersPerSec, double ff, boolean onlyProvidedFF) {}

	/**
	 * Sets the turn motors to the given rotation.
	 *
	 * @param rotation The rotation.
	 */
	public default void runTurnPosition(Rotation2d rotation) {}

	/**
	 * Resets the drive encoders.
	 */
	public default void resetEncoders() {}
}
