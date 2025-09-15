package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** The IO interface of a gyroscope for hardware interactions. */
public interface GyroIO {

	/** The inputs class for a Gyro. */
	@AutoLog
	public static class GyroIOInputs {
		public double yawDegrees;
		public double omegaDegreesPerSecond;

		public double[] odometryYawTimestamps = new double[] {};
		public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
	}

	/**
	 * Updates the given gyro inputs.
	 *
	 * @param inputs The inputs.
	 */
	public default void updateInputs(GyroIOInputsAutoLogged inputs) {};

	/**
	 * Returns the yaw of the robot.
	 *
	 * @return The yaw, in degrees.
	 */
	public default double getYawDegrees() { return 0; };

	/**
	 * Returns the angular velocity of the robot.
	 *
	 * @return The angular velocity, in degrees per second.
	 */
	public default double getOmegaDegreesPerSecond() { return 0; };

	/**
	 * Sets the yaw read by the gyro.
	 *
	 * @param yaw The yaw, in degrees.
	 */
	public default void setYaw(double yaw) {};
}
