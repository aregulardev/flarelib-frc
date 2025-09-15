package frc.robot;

import com.flarerobotics.lib.auto.AutonomousManager;
import com.flarerobotics.lib.auto.AutonomousManager.PathfindStrategy;
import com.flarerobotics.lib.auto.AutonomousManager.PrimaryCommandSource;
import com.flarerobotics.lib.subsystem.vision.LibVisionIO;
import com.flarerobotics.lib.subsystem.vision.io.LibVisionIOLimelight;
import com.flarerobotics.lib.subsystem.vision.io.LibVisionIOPhotonVisionSim;
import com.flarerobotics.lib.utils.AllianceUtil;
import com.flarerobotics.lib.utils.VisionUtils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveConstants.AutoConstants;
import frc.robot.subsystems.drive.SwerveConstants.DriveConstants;
import frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs;
import frc.robot.subsystems.drive.SwerveConstants.SwerveSimulationConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.VisionConstants;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.module.SwerveModuleIO;
import frc.robot.subsystems.drive.module.SwerveModuleIOSim;
import frc.robot.subsystems.drive.module.SwerveModuleIOSparkMax;
import frc.robot.subsystems.drive.odometry.SparkOdometryThread;
import java.util.Map;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * The main robot container.
 *
 * <pre>
 * Keybinds:
 *  - Movement (Driver Controller):
 *       * Left Stick   :: Move Chassis
 *       * Right Stick  :: Rotate Chassis
 *       * Options      :: Zero Gyro
 *       * Circle       :: Slow Mode
 *       * Cross        :: X-Lock Chassis
 * </pre>
 */
public class RobotContainer {
	private static RobotContainer instance;

	// Subsystems
	public SwerveSubsystem drive;
	public SwerveDriveSimulation driveSim;

	// Operator interface
	private CommandPS5Controller m_driverController = new CommandPS5Controller(OIConstants.kDriverControllerPort);
	// private CommandXboxController m_operatorController = new
	// CommandXboxController(OIConstants.kOperatorControllerPort);

	private AutonomousManager m_autoManager;

	// Tests
	private LoggedDashboardChooser<Supplier<Command>> m_testChooser = new LoggedDashboardChooser<>("Test Chooser");

	// Must initialize the thread before any modules
	private SparkOdometryThread m_odometryThread = new SparkOdometryThread();

	/** Constructs a new RobotContainer. */
	public RobotContainer() {
		// Initialize subsystems based on robot mode
		initSwerve();

		// Register named commands after subsystems
		registerNamedCommands();

		// Autonomous manager
		initAutoManager();

		// Robot identification routines
		createTests();

		// Register OI keybinds
		attachKeybinds();

		instance = this;
	}

	/** Registers all keybinds. */
	private void attachKeybinds() {
		// m_drive.setDefaultCommand(new RunCommand(
		// () -> m_drive.runJoystick(
		// -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
		// -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
		// -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), true),
		// m_drive));

		// Chassis keybinds
		drive.setDefaultCommand(DriveCommands.joystickDrive(drive,
				() -> MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
				() -> MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
				() -> MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)));

		// Zero gyro by aligning to the alliance driver station with the robot facing away in the case
		// of too much gyro drift
		m_driverController.options().onTrue(Commands.runOnce(drive::zeroHeadingFieldRelative).ignoringDisable(true));
		// Toggle slow mode
		m_driverController.circle().onTrue(Commands.runOnce(() -> drive.setSlowMode(!drive.isSlowMode())));
		// Stop the robot
		m_driverController.cross().onTrue(Commands.run(drive::setX));
	}

	/** Registers the NamedCommands for PathPlanner. */
	private void registerNamedCommands() {
		// Add the named commands here for PP
	}

	/** Creates robot identification tests. */
	private void createTests() {
		m_testChooser.addDefaultOption("None", () -> Commands.none());
		m_testChooser.addOption("Swerve Drive Feedforward Characterization",
				() -> DriveCommands.feedforwardCharacterization(drive));
		m_testChooser.addOption("Swerve Drive Skew Coefficient Characterization",
				() -> DriveCommands.skewCharacterization(drive));
		m_testChooser.addOption("Swerve Drive Wheel Radius Characterization",
				() -> DriveCommands.wheelRadiusCharacterization(drive));
		m_testChooser.addOption("Swerve Drive Gather SysId Data", () -> DriveCommands.sysIdFullTest(drive,
				SwerveConstants.kDynamicTestTimeLimit, SwerveConstants.kQuasistaticTestTimeLimit));
	}

	/** Initializes the autonomous routine manager. */
	private void initAutoManager() {
		m_autoManager = new AutonomousManager(Map.of("testPose", new Pose2d(3.656, 5.151, Rotation2d.fromDegrees(-60))),
				Map.of(), AutoConstants.kPathConstraints, true);
		m_autoManager.initCommandAutoChooser();
		m_autoManager.setPrimaryCommandSource(PrimaryCommandSource.kChooserCommand);
		m_autoManager.setPathfindStrategy(PathfindStrategy.kDefault);
	}

	// Mechanism Initializations //

	/** Initializes the chassis. */
	private void initSwerve() {
		switch (Robot.kRobotMode) {
			case kReal:
				driveSim = null;
				Gyro gyro = new Gyro(new GyroIOPigeon2(DriveConstants.kPigeon2CanId));

				drive = new SwerveSubsystem(new SwerveModuleIOSparkMax(0), new SwerveModuleIOSparkMax(1),
						new SwerveModuleIOSparkMax(2), new SwerveModuleIOSparkMax(3), gyro, m_odometryThread, null,
						VecBuilder.fill(0.1, 0.1, 0.05),
						new LibVisionIOLimelight("limelight", () -> Rotation2d.fromDegrees(gyro.getInputs().yawDegrees),
								VisionConstants.kRobotToCamera));
				drive.ArbitraryPIDx = new PIDController(ModuleConfigs.kDriveP, 0, ModuleConfigs.kDriveD);
				drive.ArbitraryPIDy = new PIDController(ModuleConfigs.kDriveP, 0, ModuleConfigs.kDriveD);
				drive.ArbitraryPIDAngular = new PIDController(ModuleConfigs.kTurnP, 0, ModuleConfigs.kTurnD);
				drive.ArbitraryPIDAngular.enableContinuousInput(-Math.PI, Math.PI);
				break;

			case kSim:
				driveSim = new SwerveDriveSimulation(SwerveSimulationConstants.kSwerveSimConfig,
						AllianceUtil.flipWithAlliance(Constants.kStartingPose));
				SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

				var cameraProperties = VisionUtils.getCalibrated(640, 480,
						Rotation2d.fromDegrees(VisionUtils.diagonalFovDeg(62.5, 48.9)),
						VecBuilder.fill(0, 0, 0, 0, 0, 0, 0, 0));
				cameraProperties.setFPS(20);
				cameraProperties.setAvgLatencyMs(25);
				cameraProperties.setLatencyStdDevMs(8);
				cameraProperties.setCalibError(0.01, 0.01);

				drive = new SwerveSubsystem(new SwerveModuleIOSim(driveSim.getModules()[0]),
						new SwerveModuleIOSim(driveSim.getModules()[1]),
						new SwerveModuleIOSim(driveSim.getModules()[2]),
						new SwerveModuleIOSim(driveSim.getModules()[3]),
						new Gyro(new GyroIOSim(driveSim.getGyroSimulation())), m_odometryThread,
						driveSim::setSimulationWorldPose, VecBuilder.fill(0.1, 0.1, 0.05),
						new LibVisionIOPhotonVisionSim("main", Transform3d.kZero, driveSim::getSimulatedDriveTrainPose,
								cameraProperties));
				drive.ArbitraryPIDx = new PIDController(ModuleConfigs.kDrivePSim, 0, ModuleConfigs.kDriveDSim);
				drive.ArbitraryPIDy = new PIDController(ModuleConfigs.kDrivePSim, 0, ModuleConfigs.kDriveDSim);
				drive.ArbitraryPIDAngular = new PIDController(ModuleConfigs.kTurnPSim, 0, ModuleConfigs.kTurnDSim);
				drive.ArbitraryPIDAngular.enableContinuousInput(-Math.PI, Math.PI);

				SimulatedArena.getInstance().resetFieldForAuto();
				break;

			case kReplay:
				driveSim = null;
				// Dummy IOs for replay
				drive = new SwerveSubsystem(new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {},
						new SwerveModuleIO() {}, new Gyro(new GyroIO() {}), m_odometryThread, null,
						VecBuilder.fill(0.1, 0.1, 0.05), new LibVisionIO() {});
			default:
				break;
		}

		// Zero chassis
		drive.resetOdometry(AllianceUtil.flipWithAlliance(Constants.kStartingPose));
		drive.zeroHeadingFieldRelative();
	}

	/**
	 * Returns the AutonomousManager.
	 *
	 * @return The autonomous manager
	 */
	public AutonomousManager getAutoManager() { return m_autoManager; }

	/**
	 * Returns the current identification command. Returns an empty command if comp.
	 *
	 * @return The current identification command.
	 */
	public Command getTestCommand() {
		if (Constants.kIsCompetition) return Commands.none();
		return m_testChooser.get().get();
	}

	/**
	 * Returns the singleton instance of the RobotContainer. Returns null if called before a
	 * RobotContainer is instantiated.
	 *
	 * @return The singleton instance of the RobotContainer.
	 */
	public RobotContainer getInstance() { return instance; }
}
