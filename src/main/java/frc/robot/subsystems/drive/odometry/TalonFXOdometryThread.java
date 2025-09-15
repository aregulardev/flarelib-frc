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

// A slightly modified version of SparkOdometryThread.java.

package frc.robot.subsystems.drive.odometry;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
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
 */
public class TalonFXOdometryThread implements OdometryThread<TalonFX, BaseStatusSignal, DoubleSupplier> {
	private final List<TalonFX> m_talons = new ArrayList<>();
	private final List<BaseStatusSignal> m_talonSignals = new ArrayList<>();
	private final List<DoubleSupplier> m_genericSignals = new ArrayList<>();
	private final List<Queue<Double>> m_talonQueues = new ArrayList<>();
	private final List<Queue<Double>> m_genericQueues = new ArrayList<>();
	private final List<Queue<Double>> m_timestampQueues = new ArrayList<>();

	private Notifier m_notifier = new Notifier(this::run);

	private static final double kFrquency = Robot.kRobotMode == RobotMode.kSim
			? SimulatedArena.getSimulationDt().in(Seconds) : SwerveConstants.kOdometryFrequencyHz;

	/** Constructs a new TalonFXOdometryThread. */
	public TalonFXOdometryThread() {
		if (OdometryThread.instance.get() != null)
			throw new IllegalStateException("can't have more than 1 instance of a high-frequency odometry thread");
		m_notifier.setName("HighFrequencyOdometryThread_Talon");
		OdometryThread.instance.set(this);
		OdometryThread.threadType.set("TALONFX");
	}

	@Override
	public void start() {
		if (!m_timestampQueues.isEmpty()) m_notifier.startPeriodic(1.0 / kFrquency);
	}

	@Override
	public Queue<Double> registerSignal(TalonFX motor, BaseStatusSignal signal) {
		BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(kFrquency), signal);
		Queue<Double> queue = new ArrayBlockingQueue<>(20);
		SwerveSubsystem.odometryLock.lock();
		try {
			m_talons.add(motor);
			m_talonSignals.add(signal);
			m_talonQueues.add(queue);
		} finally {
			SwerveSubsystem.odometryLock.unlock();
		}
		return queue;
	}

	@Override
	public Queue<Double> registerSignal(DoubleSupplier signal) {
		Queue<Double> queue = new ArrayBlockingQueue<>(100);
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
		Queue<Double> queue = new ArrayBlockingQueue<>(100);
		SwerveSubsystem.odometryLock.lock();
		try {
			m_timestampQueues.add(queue);
		} finally {
			SwerveSubsystem.odometryLock.unlock();
		}
		return queue;
	}

	private void run() {
		// Save new data to queues
		SwerveSubsystem.odometryLock.lock();
		try {
			// Refresh all Talon signals
			BaseStatusSignal.refreshAll(m_talonSignals.toArray(new BaseStatusSignal[m_talonSignals.size()]));
			// Get sample timestamp
			double timestamp = Timer.getFPGATimestamp();

			// Read Talon values
			double[] talonValues = new double[m_talonSignals.size()];
			for (int i = 0; i < m_talonSignals.size(); i++)
				talonValues[i] = m_talonSignals.get(i).getValueAsDouble();

			// Add values to queues
			for (int i = 0; i < m_talonSignals.size(); i++)
				m_talonQueues.get(i).offer(talonValues[i]);
			for (int i = 0; i < m_genericSignals.size(); i++)
				m_genericQueues.get(i).offer(m_genericSignals.get(i).getAsDouble());
			for (int i = 0; i < m_timestampQueues.size(); i++)
				m_timestampQueues.get(i).offer(timestamp);
		} finally {
			SwerveSubsystem.odometryLock.unlock();
		}
	}
}
