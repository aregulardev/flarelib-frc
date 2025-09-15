package frc.robot.subsystems.drive.odometry;

import java.util.Queue;
import java.util.concurrent.atomic.AtomicReference;

/** The interface for an odometry thread. */
public interface OdometryThread<Motor, MotorSignalSupplier, GenericSignalSupplier> {
	// Not the best practice
	public static AtomicReference<OdometryThread<?, ?, ?>> instance = new AtomicReference<>();
	public static AtomicReference<String> threadType = new AtomicReference<>();

	public static OdometryThread<?, ?, ?> getInstance() { return instance.get(); }

	/**
	 * Starts the odometry thread.
	 */
	public void start();

	/**
	 * Registers a motor signal to be read from the thread.
	 *
	 * @param motor  The motor the signal was obtained from.
	 * @param signal The signal.
	 * @return The queue containing the measured signals.
	 */
	public Queue<Double> registerSignal(Motor motor, MotorSignalSupplier signal);

	/**
	 * Registers a generic signal to be read from the thread.
	 *
	 * @param signal The generic signal.
	 * @return The queue containing the measured samples.
	 */
	public Queue<Double> registerSignal(GenericSignalSupplier signal);

	/**
	 * Returns a new queue that contains timestamp values for each sample.
	 *
	 * @return The queue.
	 */
	public Queue<Double> makeTimestampQueue();
}
