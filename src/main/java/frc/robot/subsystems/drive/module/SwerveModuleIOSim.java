package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kDriveDSim;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kDriveISim;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kDriveKsSim;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kDriveKvSim;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kDriveMotorCurrentLimit;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kDrivePSim;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kTurnDSim;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kTurnISim;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kTurnMotorCurrentLimit;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kTurnPSim;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConstants.kWheelDiameterMeters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** The swerve module IO implementation for simulation. */
public class SwerveModuleIOSim implements SwerveModuleIO {
	private SwerveModuleSimulation m_moduleSimulation;
	private SimulatedMotorController.GenericMotorController m_driveMotor;
	private SimulatedMotorController.GenericMotorController m_turnMotor;

	private final PIDController m_driveController = new PIDController(kDrivePSim, kDriveISim, kDriveDSim);
	private final PIDController m_turnController = new PIDController(kTurnPSim, kTurnISim, kTurnDSim);

	private boolean m_driveClosedLoop = false;
	private boolean m_turnClosedLoop = false;

	private double m_driveVoltage = 0.0;
	private double m_driveFF = 0.0;
	private double m_turnVoltage = 0.0;

	/**
	 * Constructs a new SwerveModuleIOSim.
	 *
	 * @param moduleSimulation The module simulation obtained from the drivetrain simulation.
	 */
	public SwerveModuleIOSim(SwerveModuleSimulation moduleSimulation) {
		m_moduleSimulation = moduleSimulation;
		m_driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
				.withCurrentLimit(Amps.of(kDriveMotorCurrentLimit));
		m_turnMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(kTurnMotorCurrentLimit));

		// Enable angle wrapping for turning
		m_turnController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		// Handle closed-loop state, reset otherwise
		if (m_driveClosedLoop) m_driveVoltage = m_driveFF + m_driveController.calculate(
				m_moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond) * kWheelDiameterMeters / 2);
		else m_driveController.reset();

		if (m_turnClosedLoop)
			m_turnVoltage = m_turnController.calculate(m_moduleSimulation.getSteerAbsoluteFacing().getRadians());
		else m_turnController.reset();

		// Run voltages
		m_driveMotor.requestVoltage(Volts.of(m_driveVoltage));
		m_turnMotor.requestVoltage(Volts.of(m_turnVoltage));

		// Update drive inputs
		inputs.driveMotorConnected = true;
		inputs.drivePositionMeters = m_moduleSimulation.getDriveWheelFinalPosition().in(Radians) * kWheelDiameterMeters
				/ 2;
		inputs.driveVelocityMetersPerSec = m_moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond)
				* kWheelDiameterMeters / 2;
		inputs.driveAppliedVolts = m_driveVoltage;
		inputs.driveSupplyCurrentAmps = m_moduleSimulation.getDriveMotorSupplyCurrent().in(Amps);
		inputs.driveStatorCurrentAmps = m_moduleSimulation.getDriveMotorStatorCurrent().in(Amps);
		inputs.driveTemperatureCelsius = 20;

		// Update turn inputs
		inputs.turnMotorConnected = true;
		inputs.turnPosition = m_moduleSimulation.getSteerAbsoluteFacing();
		inputs.turnVelocityRadPerSec = m_moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
		inputs.turnAppliedVolts = m_turnVoltage;
		inputs.turnSupplyCurrentAmps = m_moduleSimulation.getSteerMotorSupplyCurrent().in(Amps);
		inputs.turnStatorCurrentAmps = m_moduleSimulation.getSteerMotorStatorCurrent().in(Amps);
		inputs.turnTemperatureCelsius = 20;

		// Update odometry inputs
		inputs.odometryTimestamps = getSimulationOdometryTimeStamps();
		inputs.odometryDrivePositionsMeters = Arrays.stream(m_moduleSimulation.getCachedDriveWheelFinalPositions())
				.mapToDouble(angle -> angle.in(Radians) * kWheelDiameterMeters / 2)
				.toArray();
		inputs.odometryTurnPositions = m_moduleSimulation.getCachedSteerAbsolutePositions();
	}

	@Override
	public void runDriveVelocity(double velocityMetersPerSec) {
		runDriveVelocityFF(velocityMetersPerSec, 0);
	}

	@Override
	public void runDriveVelocityFF(double velocityMetersPerSec, double ff) {
		m_driveClosedLoop = true;
		m_driveFF = ff + kDriveKsSim * Math.signum(velocityMetersPerSec) + kDriveKvSim * velocityMetersPerSec;
		m_driveController.setSetpoint(velocityMetersPerSec);
	}

	@Override
	public void runDriveVelocityFF(double velocityMetersPerSec, double ff, boolean onlyProvidedFF) {
		m_driveClosedLoop = true;
		m_driveFF = ff + (onlyProvidedFF ? 0
				: kDriveKsSim * Math.signum(velocityMetersPerSec) + kDriveKvSim * velocityMetersPerSec);
		m_driveController.setSetpoint(velocityMetersPerSec);
	}

	@Override
	public void runDriveOpenLoop(double output) {
		m_driveClosedLoop = false;
		m_driveVoltage = output;
	}

	@Override
	public void runTurnPosition(Rotation2d rotation) {
		m_turnClosedLoop = true;
		m_turnController.setSetpoint(rotation.getRadians());
	}

	@Override
	public void runTurnOpenLoop(double output) {
		m_turnClosedLoop = false;
		m_turnVoltage = output;
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
