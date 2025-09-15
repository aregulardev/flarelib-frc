package frc.robot.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/** The gyroscope IO implementation for simulation. */
public class GyroIOSim implements GyroIO {
	private GyroSimulation m_sim;

	/**
	 * Constructs a new GyroIOSim.
	 *
	 * @param sim The gyro simulation instance to use, obtained from the drivetrain simulation.
	 */
	public GyroIOSim(GyroSimulation sim) {
		m_sim = sim;
	}

	@Override
	public void updateInputs(GyroIOInputsAutoLogged inputs) {
		inputs.yawDegrees = getYawDegrees();
		inputs.omegaDegreesPerSecond = getOmegaDegreesPerSecond();

		inputs.odometryYawTimestamps = getSimulationOdometryTimeStamps();
		inputs.odometryYawPositions = m_sim.getCachedGyroReadings();
	}

	@Override
	public double getOmegaDegreesPerSecond() { return m_sim.getMeasuredAngularVelocity().in(DegreesPerSecond); }

	@Override
	public double getYawDegrees() { return m_sim.getGyroReading().getDegrees(); }

	@Override
	public void setYaw(double yaw) {
		m_sim.setRotation(Rotation2d.fromDegrees(yaw));
	}

	private static double[] getSimulationOdometryTimeStamps() {
		final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
		for (int i = 0; i < odometryTimeStamps.length; i++)
			// odometryTimeStamps[i] = Timer.getFPGATimestamp() - 0.02 + i *
			// SimulatedArena.getSimulationDt().in(Seconds);
			odometryTimeStamps[i] = Timer.getFPGATimestamp() - Constants.kLoopPeriodSeconds
					+ i * SimulatedArena.getSimulationDt().in(Seconds);

		return odometryTimeStamps;
	}
}
