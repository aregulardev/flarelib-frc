package frc.robot.subsystems.drive.module;

import static frc.robot.Robot.kRobotMode;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kCosineCompensateReal;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kCosineCompensateSim;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConstants.kDriveMotorMaxTemperature;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConstants.kEmergencyStopOnOverheat;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConstants.kTurnMotorMaxTemperature;

import com.flarerobotics.lib.utils.Tracer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.RobotMode;
import frc.robot.subsystems.drive.SwerveConstants.DriveConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The class representing an individual swerve module.
 *
 * <p>
 * <b>Note</b>: The odometry thread must be initialized before any module instantiations.
 */
public class SwerveModule {
	private static final int kDSOverheatSpamCount = 10;

	private SwerveModuleIO m_io;
	private SwerveModuleIOInputsAutoLogged m_inputs = new SwerveModuleIOInputsAutoLogged();

	private int m_idx;

	private SwerveModulePosition m_position = new SwerveModulePosition();
	private SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[] {};

	private Alert m_driveDisconnectAlert, m_steerDisconnectAlert;
	private Alert m_driveTempAlert, m_steerTempAlert;

	private boolean m_overheatEStop = false;

	/**
	 * Constructs a new SwerveModule.
	 *
	 * @param io  The IO interface for the module.
	 * @param idx The module ID. 0 = front left, 1 = front right, 2 = back left, 3 = back right.
	 */
	public SwerveModule(SwerveModuleIO io, int idx) {
		m_io = io;
		m_idx = idx;

		m_driveTempAlert = new Alert("Swerve module #" + m_idx + " drive motor temperature exceeded temperature limit!",
				AlertType.kWarning);
		m_steerTempAlert = new Alert("Swerve module #" + m_idx + " steer motor temperature exceeded temperature limit!",
				AlertType.kWarning);

		m_driveDisconnectAlert = new Alert("Swerve module #" + m_idx + " drive motor has disconnected!",
				AlertType.kWarning);
		m_steerDisconnectAlert = new Alert("Swerve module #" + m_idx + " steer motor has disconnected!",
				AlertType.kWarning);
	}

	/** Applies periodic updates. */
	public void periodic() {
		Tracer.start("SwerveModule#" + m_idx + "_Periodic");

		m_io.updateInputs(m_inputs);
		Logger.processInputs("Drive/Module" + m_idx, m_inputs);

		m_position = new SwerveModulePosition(m_inputs.drivePositionMeters, m_inputs.turnPosition);

		int sampleCount = m_inputs.odometryTimestamps.length; // All signals are sampled together
		m_modulePositions = new SwerveModulePosition[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			double positionMeters = m_inputs.odometryDrivePositionsMeters[i];
			Rotation2d angle = m_inputs.odometryTurnPositions[i];
			m_modulePositions[i] = new SwerveModulePosition(positionMeters, angle);
		}

		m_driveDisconnectAlert.set(!m_inputs.driveMotorConnected);
		m_steerDisconnectAlert.set(!m_inputs.turnMotorConnected);

		// Update the module temperature alerts

		// Drive temp limit
		if (m_inputs.driveTemperatureCelsius > kDriveMotorMaxTemperature && m_driveTempAlert.get() == false) {
			m_overheatEStop = kEmergencyStopOnOverheat;
			if (m_overheatEStop) stop();
			m_driveTempAlert.set(true);
			// Spam DS log to get attention
			for (int i = 0; i < kDSOverheatSpamCount; i++)
				DriverStation.reportWarning(
						"WARNING: Swerve module #" + m_idx
								+ " drive motor temperature exceeded temperature limit! Please stop the drive motor.",
						false);
		} else {
			m_driveTempAlert.set(false);
		}

		// Steer temp limit
		if (m_inputs.turnTemperatureCelsius > kTurnMotorMaxTemperature && m_steerTempAlert.get() == false) {
			m_overheatEStop = kEmergencyStopOnOverheat;
			if (m_overheatEStop) stop();
			m_steerTempAlert.set(true);
			for (int i = 0; i < kDSOverheatSpamCount; i++)
				// Spam DS log to get attention
				DriverStation.reportWarning(
						"WARNING: Swerve module #" + m_idx
								+ " steer motor temperature exceeded temperature limit! Please stop the steer motor.",
						false);
		} else {
			m_steerTempAlert.set(false);
		}

		// No overheating
		if (!m_driveTempAlert.get() && !m_steerTempAlert.get()) m_overheatEStop = false;

		Tracer.finish("SwerveModule#" + m_idx + "_Periodic");
	}

	/**
	 * Sets the target state of the module. If stopped due to overheating, won't apply the target
	 * state.
	 *
	 * @param desiredState The target state.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		if (m_overheatEStop) { stop(); return; }
		// Optimize state
		optimize(desiredState);

		if (Math.abs(desiredState.speedMetersPerSecond) < DriveConstants.kJitterVelocityDeadbandMps) { stop(); return; }

		m_io.runDriveVelocity(desiredState.speedMetersPerSecond);
		m_io.runTurnPosition(desiredState.angle);
	}

	/**
	 * Sets the target state of the module with the given feedforward. If stopped due to
	 * overheating, won't apply the target state.
	 *
	 * @param desiredState The target state.
	 * @param ff           The feedforward voltage.
	 */
	public void setDesiredStateFF(SwerveModuleState desiredState, double ff) {
		if (m_overheatEStop) { stop(); return; }

		// Optimize state
		optimize(desiredState);

		if (Math.abs(desiredState.speedMetersPerSecond) < DriveConstants.kJitterVelocityDeadbandMps) { stop(); return; }

		m_io.runDriveVelocityFF(desiredState.speedMetersPerSecond, ff);
		m_io.runTurnPosition(desiredState.angle);
	}

	/**
	 * Sets the target state of the module with the given feedforward. If stopped due to
	 * overheating, won't apply the target state.
	 *
	 * @param desiredState   The target state.
	 * @param ff             The feedforward voltage.
	 * @param onlyProvidedFF Whether to only use the provided feedforward voltage or not.
	 */
	public void setDesiredStateFF(SwerveModuleState desiredState, double ff, boolean onlyProvidedFF) {
		if (m_overheatEStop) { stop(); return; }

		// Optimize state
		optimize(desiredState);

		if (Math.abs(desiredState.speedMetersPerSecond) < DriveConstants.kJitterVelocityDeadbandMps) { stop(); return; }

		m_io.runDriveVelocityFF(desiredState.speedMetersPerSecond, ff, onlyProvidedFF);
		m_io.runTurnPosition(desiredState.angle);
	}

	/**
	 * Sets the module to drive in open-loop mode.
	 *
	 * @param voltsDrive The voltage to apply to the drive motor.
	 * @param voltsTurn  The voltage to apply to the steer motor.
	 */
	public void runOpenLoop(double voltsDrive, double voltsTurn) {
		if (m_overheatEStop) { stop(); return; }
		m_io.runDriveOpenLoop(voltsDrive);
		m_io.runTurnOpenLoop(voltsTurn);
	}

	/**
	 * Resets the drive encoders.
	 */
	public void resetEncoders() {
		m_io.resetEncoders();
	}

	/**
	 * Stops the module.
	 */
	public void stop() {
		m_io.runDriveOpenLoop(0);
		m_io.runTurnOpenLoop(0);
	}

	/**
	 * Runs system characterization.
	 *
	 * @param output The voltage to apply.
	 */
	public void runCharacterization(double output) {
		if (m_overheatEStop) { stop(); return; }
		m_io.runDriveOpenLoop(output);
		m_io.runTurnPosition(Rotation2d.kZero);
	}

	/**
	 * Returns the current odometry-estimated module position.
	 *
	 * @return The module position.
	 */
	public SwerveModulePosition getOdometryPosition() { return m_position; }

	/**
	 * Returns the current module state.
	 *
	 * @return The current module state.
	 */
	public SwerveModuleState getState() { return new SwerveModuleState(getDriveVelocity(), getRotation()); }

	/**
	 * Returns the current drive velocity, in m/s.
	 *
	 * @return The velocity.
	 */
	public double getDriveVelocity() { return m_inputs.driveVelocityMetersPerSec; }

	/**
	 * Returns the current IO inputs.
	 *
	 * @return The current IO inputs.
	 */
	public SwerveModuleIOInputsAutoLogged getInputs() { return m_inputs; }

	/**
	 * Returns the sampled swerve module positions.
	 *
	 * @return The samples swerve module positions.
	 */
	public SwerveModulePosition[] getOdometryPositions() { return m_modulePositions; }

	/**
	 * Returns the current rotation of the module.
	 *
	 * @return The current rotation of the module.
	 */
	public Rotation2d getRotation() { return m_inputs.turnPosition; }

	private void optimize(SwerveModuleState unoptimized) {
		unoptimized.optimize(getRotation());
		if ((kRobotMode == RobotMode.kSim && kCosineCompensateSim)
				|| (kRobotMode == RobotMode.kReal && kCosineCompensateReal))
			unoptimized.cosineScale(getRotation());
	}
}
