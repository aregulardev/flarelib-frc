// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import com.flarerobotics.lib.container.LoggedTunableNumber;
import com.flarerobotics.lib.sim.HardwareSimUtils;
import com.flarerobotics.lib.utils.AllianceUtil;
import com.flarerobotics.lib.utils.Tracer;
import com.flarerobotics.lib.utils.Utils;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.PhysicalParameters;
import frc.robot.Constants.RobotMode;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** The main Robot class. */
public class Robot extends LoggedRobot {
	private final RobotContainer m_robotContainer;
	public static final RobotMode kRobotMode = Constants.kIsReplay ? RobotMode.kReplay
			: (Robot.isReal() ? RobotMode.kReal : RobotMode.kSim);

	private Command m_testCommand;
	private Runnable m_scheduleLogger;

	// Alerts
	private Alert m_canBusAlert = new Alert("A CAN Bus (bus=rio) error has occured!", Alert.AlertType.kError);
	private Alert m_batteryAlert = new Alert(
			"Battery voltage is too low when the robot is disabled! Replace the battery if possible.",
			Alert.AlertType.kWarning);

	static {
		if (kRobotMode == RobotMode.kSim) {
			// Default station
			DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
			DriverStationSim.notifyNewData();
		}
	}

	/** Constructs a new Robot. */
	public Robot() {
		LoggedTunableNumber.DoTuning = Constants.kDoTuning;
		m_robotContainer = new RobotContainer();
	}

	@Override
	public void robotInit() {
		// Record metadata
		Logger.recordMetadata("IsCompetition", String.valueOf(Constants.kIsCompetition));

		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		switch (BuildConstants.DIRTY) {
			case 0:
				Logger.recordMetadata("GitDirty", "All changes committed");
				break;

			case 1:
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;

			default:
				Logger.recordMetadata("GitDirty", "Unknown");
				break;
		}

		// Mode and logger settings
		switch (kRobotMode) {
			case kReal:
				Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
				Logger.addDataReceiver(new NT4Publisher());
				break;

			case kSim:
				Logger.addDataReceiver(new NT4Publisher());
				// Have to use this way because MapleSim doesn't have a way of changing the nominal voltage
				SimulatedBattery
						.addElectricalAppliances(() -> Amps.of(1 / 0.02 * (13.5 - PhysicalParameters.kNominalVoltage)));
				// Decrease the linear filter's taps
				HardwareSimUtils.modifySimulatedBatteryFilter(5);
				break;

			case kReplay:
				// Replaying a log, set up replay source
				setUseTiming(false); // Run as fast as possible
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
				break;

			default:
				throw new RuntimeException("Invalid robot mode: " + kRobotMode);
		}
		Logger.start();

		PathfindingCommand.warmupCommand().schedule();
		m_scheduleLogger = Utils.registerLoopTimeLogger(this);
	}

	@Override
	public void robotPeriodic() {
		// For thread priority (from docs):
		// This customization should only be used if the loop cycle time is significantly less than
		// 20ms, which allows other threads to continue running between user code cycles. We always
		// recommend thoroughly testing this change to ensure that it does not cause unintended side
		// effects (examples include NetworkTables lag, CAN timeouts, etc). In general, this
		// customization is only recommended for advanced users who understand the potential
		// side-effects.

		// Switch thread to high priority to improve loop timing
		// Threads.setCurrentThreadPriority(true, 99);

		Tracer.start("CommandScheduler");
		CommandScheduler.getInstance().run();
		Tracer.finish("CommandScheduler");

		Tracer.start("RobotPeriodic_Misc");
		if (kRobotMode == RobotMode.kSim) SmartDashboard.putNumber("Battery Voltage", RoboRioSim.getVInVoltage());
		else SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

		// Update alerts
		var canStatus = RobotController.getCANStatus();
		m_canBusAlert.set(canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0);

		m_batteryAlert.set(isDisabled()
				&& RobotController.getBatteryVoltage() <= PhysicalParameters.kRobotWarningBatteryVoltageThreshold);

		m_scheduleLogger.run();

		Tracer.finish("RobotPeriodic_Misc");

		// Return to normal thread priority
		// Threads.setCurrentThreadPriority(false, 10);
	}

	@Override
	public void disabledInit() {
		// Lock wheels to prevent movement
		m_robotContainer.drive.setX();

		// Reset if in sim
		if (kRobotMode == RobotMode.kSim) {
			m_robotContainer.drive.resetOdometry(AllianceUtil.flipWithAlliance(Constants.kStartingPose));
			SimulatedArena.getInstance().resetFieldForAuto();
		}
	}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		// Cancel all commands at start of test mode
		CommandScheduler.getInstance().cancelAll();
		// Assign and schedule the test command
		CommandScheduler.getInstance().schedule(m_testCommand = m_robotContainer.getTestCommand());
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {
		if (m_testCommand != null) m_testCommand.cancel();
	}

	@Override
	public void simulationInit() {
		// Do not use with advantagekit
		// SimulatedArena.overrideSimulationTimings(Seconds.of(Constants.kLoopPeriodSeconds),
		// (int) SwerveConstants.kOdometryFrequencyHz / 50);
	}

	@Override
	public void simulationPeriodic() {
		SimulatedArena.getInstance().simulationPeriodic();

		// Log field state
		Logger.recordOutput("FieldSimulation/RobotPosition", m_robotContainer.driveSim.getSimulatedDriveTrainPose());
		Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
		Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
	}
}
