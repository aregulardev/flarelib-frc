package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.odometry.OdometryThread;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/** The gyroscope IO implementation for the Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {

	private Pigeon2 m_pigeon;

	private Queue<Double> m_odometryTimestamps;
	private Queue<Double> m_odometryYawPositions;

	/**
	 * Constructs a new GyroIOPigeon2.
	 *
	 * @param canId The CAN ID of the pigeon.
	 */
	public GyroIOPigeon2(int canId) {
		this(canId, "");
	}

	/**
	 * Constructs a new GyroIOPigeon2.
	 *
	 * @param canId  The CAN ID of the pigeon.
	 * @param canBus The CAN bus name.
	 */
	public GyroIOPigeon2(int canId, String canBus) {
		m_pigeon = new Pigeon2(canId, canBus);

		@SuppressWarnings("unchecked")
		OdometryThread<?, ?, DoubleSupplier> odometryThread = (OdometryThread<?, ?, DoubleSupplier>) OdometryThread
				.getInstance();
		m_odometryTimestamps = odometryThread.makeTimestampQueue();
		m_odometryYawPositions = odometryThread.registerSignal(this::getYawDegrees);
	}

	@Override
	public void updateInputs(GyroIOInputsAutoLogged inputs) {
		inputs.yawDegrees = getYawDegrees();
		inputs.omegaDegreesPerSecond = getOmegaDegreesPerSecond();

		inputs.odometryYawTimestamps = m_odometryTimestamps.stream().mapToDouble((v) -> v).toArray();
		inputs.odometryYawPositions = m_odometryYawPositions.stream()
				.map(Rotation2d::fromDegrees)
				.toArray(Rotation2d[]::new);

		m_odometryTimestamps.clear();
		m_odometryYawPositions.clear();
	}

	@Override
	public double getYawDegrees() { return m_pigeon.getYaw().getValueAsDouble(); }

	@Override
	public double getOmegaDegreesPerSecond() { return m_pigeon.getAngularVelocityZWorld().getValueAsDouble(); }

	@Override
	public void setYaw(double yaw) {
		m_pigeon.setYaw(yaw);
	}
}
