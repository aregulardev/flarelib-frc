// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.odometry;

import static edu.wpi.first.units.Units.Seconds;

import com.flarerobotics.lib.utils.Tracer;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.RobotMode;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of
 * queues.
 *
 * <p>
 * Includes an overload for Spark signals, which checks for errors to ensure that all
 * measurements in the sample are valid.
 */
public class SparkOdometryThread implements OdometryThread<SparkBase, DoubleSupplier, DoubleSupplier> {
	private static final int kQueueLength = 20;

	private final List<SparkBase> m_sparks = new ArrayList<>();
	private final List<DoubleSupplier> m_sparkSignals = new ArrayList<>();
	private final List<DoubleSupplier> m_genericSignals = new ArrayList<>();
	private final List<Queue<Double>> m_sparkQueues = new ArrayList<>();
	private final List<Queue<Double>> m_genericQueues = new ArrayList<>();
	private final List<Queue<Double>> m_timestampQueues = new ArrayList<>();

	private Notifier m_notifier = new Notifier(this::run);

	/** Constructs a new SparkOdometryThread. */
	public SparkOdometryThread() {
		if (OdometryThread.instance.get() != null)
			throw new IllegalStateException("can't have more than 1 instance of a high-frequency odometry thread");
		m_notifier.setName("HighFrequencyOdometryThread_Spark");
		OdometryThread.instance.set(this);
		OdometryThread.threadType.set("SPARK");
	}

	@Override
	public void start() {
		if (!m_timestampQueues.isEmpty()) m_notifier.startPeriodic(1.0 / (Robot.kRobotMode == RobotMode.kSim
				? SimulatedArena.getSimulationDt().in(Seconds) : SwerveConstants.kOdometryFrequencyHz));
	}

	@Override
	public Queue<Double> registerSignal(SparkBase motor, DoubleSupplier signal) {
		Queue<Double> queue = new ArrayBlockingQueue<>(kQueueLength);
		SwerveSubsystem.odometryLock.lock();
		try {
			m_sparks.add(motor);
			m_sparkSignals.add(signal);
			m_sparkQueues.add(queue);
		} finally {
			SwerveSubsystem.odometryLock.unlock();
		}
		return queue;
	}

	@Override
	public Queue<Double> registerSignal(DoubleSupplier signal) {
		Queue<Double> queue = new ArrayBlockingQueue<>(kQueueLength);
		SwerveSubsystem.odometryLock.lock();
		try {
			m_genericSignals.add(signal);
			m_genericQueues.add(queue);
		} finally {
			SwerveSubsystem.odometryLock.unlock();
		}
		return queue;
	}

	@Override
	public Queue<Double> makeTimestampQueue() {
		Queue<Double> queue = new ArrayBlockingQueue<>(kQueueLength);
		SwerveSubsystem.odometryLock.lock();
		try {
			m_timestampQueues.add(queue);
		} finally {
			SwerveSubsystem.odometryLock.unlock();
		}
		return queue;
	}

	private void run() {
		Tracer.start("HighFreqOdometryThread_Spark_Run");
		// Save new data to queues
		SwerveSubsystem.odometryLock.lock();
		try {
			// Get sample timestamp
			double timestamp = Timer.getFPGATimestamp();

			// Read Spark values, mark invalid in case of error
			double[] sparkValues = new double[m_sparkSignals.size()];
			boolean isValid = true;
			for (int i = 0; i < m_sparkSignals.size(); i++) {
				sparkValues[i] = m_sparkSignals.get(i).getAsDouble();
				// Check for errors in any spark
				if (m_sparks.get(i).getLastError() != REVLibError.kOk) isValid = false;
			}

			// If valid, add values to queues
			if (isValid) {
				for (int i = 0; i < m_sparkSignals.size(); i++)
					m_sparkQueues.get(i).offer(sparkValues[i]);
				for (int i = 0; i < m_genericSignals.size(); i++)
					m_genericQueues.get(i).offer(m_genericSignals.get(i).getAsDouble());
				for (int i = 0; i < m_timestampQueues.size(); i++)
					m_timestampQueues.get(i).offer(timestamp);
			}
		} finally {
			SwerveSubsystem.odometryLock.unlock();
		}

		Tracer.finish("HighFreqOdometryThread_Spark_Run");
	}
}
