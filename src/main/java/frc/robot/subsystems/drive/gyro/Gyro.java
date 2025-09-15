package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.Logger;

/** A class representing a gyro. */
public class Gyro {
	private GyroIO m_io;
	private GyroIOInputsAutoLogged m_inputs = new GyroIOInputsAutoLogged();

	/**
	 * Constructs a new Gyro.
	 *
	 * @param io The IO interface for hardware interactions.
	 */
	public Gyro(GyroIO io) {
		m_io = io;
	}

	/** Applies periodic updates. */
	public void periodic() {
		m_io.updateInputs(m_inputs);
		Logger.processInputs("Gyro/Inputs", m_inputs);
	}

	/**
	 * Returns the gyro inputs.
	 *
	 * @return The inputs.
	 */
	public GyroIOInputsAutoLogged getInputs() { return m_inputs; }

	/**
	 * Returns the IO interface of the gyro.
	 *
	 * @return The IO interface of the gyro.
	 */
	public GyroIO getIO() { return m_io; }
}
