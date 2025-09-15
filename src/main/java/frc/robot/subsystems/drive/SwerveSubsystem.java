package frc.robot.subsystems.drive;

import static com.flarerobotics.lib.utils.Utils.roundDigits;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kDriveKinematics;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kMaxAngularAcceleration;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kMaxLinearAcceleration;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kSkewCorrectionFactor;

import com.flarerobotics.lib.auto.LocalADStarAK;
import com.flarerobotics.lib.subsystem.vision.LibVisionIO;
import com.flarerobotics.lib.subsystem.vision.LibVisionSubsystem;
import com.flarerobotics.lib.utils.AllianceUtil;
import com.flarerobotics.lib.utils.Tracer;
import com.flarerobotics.lib.utils.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveConstants.AutoConstants;
import frc.robot.subsystems.drive.SwerveConstants.DriveConstants;
import frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs;
import frc.robot.subsystems.drive.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.drive.module.SwerveModule;
import frc.robot.subsystems.drive.module.SwerveModuleIO;
import frc.robot.subsystems.drive.odometry.OdometryThread;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * A holonomic drive subsystem.
 */
public class SwerveSubsystem extends SubsystemBase {
	// Swerve modules
	private final SwerveModule[] m_modules;

	// The gyroscope
	private final Gyro m_gyro;

	private boolean m_isSlowMode = false;

	// Vision
	private LibVisionSubsystem m_vision;
	private boolean m_registerVision = true;

	// PP config alert
	private Alert m_configAlert = new Alert("PathPlanner RobotConfig failure to load, restart robot!",
			AlertType.kError);

	// Pose tracking
	private SwerveDrivePoseEstimator m_poseEstimator;
	private Field2d m_field = new Field2d();

	public PIDController ArbitraryPIDx, ArbitraryPIDy;
	public PIDController ArbitraryPIDAngular;

	// Odometry thread
	public static Lock odometryLock = new ReentrantLock();
	public static OdometryThread<?, ?, ?> odometryThread;

	// Accelerations
	private ChassisSpeeds m_accelField; // Accel represented by a ChassisSpeeds
	private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds();
	private double m_lastTick = Timer.getFPGATimestamp();

	private SlewRateLimiter m_xLimiter = new SlewRateLimiter(kMaxLinearAcceleration),
			m_yLimiter = new SlewRateLimiter(kMaxLinearAcceleration),
			m_rotLimiter = new SlewRateLimiter(kMaxAngularAcceleration);

	private final Consumer<Pose2d> m_resetSim;

	private final SysIdRoutine m_sysId;

	/**
	 * Constructs a new SwerveSubsystem.
	 *
	 * <p>
	 * Note: Uses the default standard deviation baselines for vision.
	 *
	 * @param frontLeft           The front left swerve module.
	 * @param frontRight          The front right swerve module.
	 * @param rearLeft            The back left swerve module.
	 * @param rearRight           The back right swerve module.
	 * @param gyro                The gyroscope.
	 * @param odometry            The odometry thread.
	 * @param resetSimulationPose The callback to reset the simulated robot pose.
	 * @param odometryStdDevs     The standard deviations for odometry measurements in the form of
	 *                            a vector: {@code [x (m), y (m), theta (rads)]}.
	 * @param visionIOs           The preset vision subystem IOs for localization.
	 */
	public SwerveSubsystem(SwerveModuleIO frontLeft, SwerveModuleIO frontRight, SwerveModuleIO rearLeft,
			SwerveModuleIO rearRight, Gyro gyro, OdometryThread<?, ?, ?> odometry, Consumer<Pose2d> resetSimulationPose,
			Matrix<N3, N1> odometryStdDevs, LibVisionIO... visionIOs) {
		m_modules = new SwerveModule[] { new SwerveModule(frontLeft, 0), new SwerveModule(frontRight, 1),
				new SwerveModule(rearLeft, 2), new SwerveModule(rearRight, 3) };

		// Start odometry
		odometryThread = odometry;
		odometryThread.start();
		System.out.println("Starting odometry thread for: " + OdometryThread.threadType.get());

		m_gyro = gyro;
		// Init pose estimator, vision constants default b.c. we already set them when adding a
		// vision measurement
		m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, new Rotation2d(),
				getModulePositions(), new Pose2d(), odometryStdDevs, VecBuilder.fill(0.9, 0.9, 0.9));
		// Initialize vision if applicable
		if (visionIOs != null && visionIOs.length > 0 && visionIOs[0] != null) {
			m_vision = new LibVisionSubsystem((pose, time, stdDevs) -> {
				if (m_registerVision) m_poseEstimator.addVisionMeasurement(pose, time, stdDevs);
			}, visionIOs);
			// m_vision.setLinearStdDevBaseline(visionStdDevs.get(0, 0));
			// m_vision.setAngularStdDevBaseline(visionStdDevs.get(2, 0));
		}
		m_resetSim = resetSimulationPose;

		// Get robot config
		RobotConfig config = null;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			m_configAlert.set(true);
			DriverStation.reportError("Unable to load RobotConfig from GUI, please reboot the robot: " + e.getMessage(),
					true);
		}

		// Configure auto builder
		AutoBuilder.configure(this::getFilteredPose, this::resetOdometry, this::getChassisSpeedsRobotRelative,
				(speeds, ff) -> {
					SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

					// Set desired states
					if (AutoConstants.kUseAutonomousFeedforward) {
						Force[] forces = ff.linearForces();
						double[] feedforwards = new double[4];
						double wheelRad = ModuleConstants.kWheelDiameterMeters / 2;

						// Compute feedforward voltages for each module
						int idx = 0;
						for (Force f : forces) {
							feedforwards[idx] = ModuleConfigs.kPerModuleDriveGearbox.getVoltage(
									f.in(Newtons) * wheelRad / ModuleConstants.kDrivingMotorReduction,
									states[idx].speedMetersPerSecond / wheelRad
											* ModuleConstants.kDrivingMotorReduction);
							idx++;
						}

						// Apply the states and feedforward voltages to the modules
						for (int i = 0; i < 4; i++)
							m_modules[i].setDesiredStateFF(states[i], feedforwards[i], true);
					} else {
						setModuleStates(states);
					}
				},
				new PPHolonomicDriveController(
						new PIDConstants(AutoConstants.kPathDriveP, 0.0, AutoConstants.kPathDriveD),
						new PIDConstants(AutoConstants.kPathTurnP, 0.0, AutoConstants.kPathTurnD)),
				config,
				() -> AllianceUtil.isRedAlliance() == (AutoConstants.kInvertAutonomousPathAlliance == Alliance.Red),
				this);

		// PathPlanner trajectory logging
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
			m_field.getObject("Trajectory").setPoses(activePath);
		});
		PathPlannerLogging.setLogTargetPoseCallback(
				(targetPose) -> { Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose); });

		// Initialize SysId
		m_sysId = new SysIdRoutine(
				new SysIdRoutine.Config(Volts.of(0.2).per(Seconds.of(1).unit()), Volts.of(3), null,
						(state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
				new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));

		// Special widgets for Elastic
		// Put field
		SmartDashboard.putData("Field", m_field);

		// Put gyro for Elastic
		SmartDashboard.putData("Chassis Rotation (deg)", new Sendable() {
			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("Gyro");
				builder.addDoubleProperty("Value", () -> MathUtil.inputModulus(getRobotRotation().getDegrees(), 0, 360),
						null);
			}
		});

		// Put module states for Elastic
		SmartDashboard.putData("Swerve Chassis", new Sendable() {
			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");

				builder.addDoubleProperty("Front Left Angle", () -> m_modules[0].getRotation().getRadians(), null);
				builder.addDoubleProperty("Front Left Velocity", () -> m_modules[0].getDriveVelocity(), null);

				builder.addDoubleProperty("Front Right Angle", () -> m_modules[1].getRotation().getRadians(), null);
				builder.addDoubleProperty("Front Right Velocity", () -> m_modules[1].getDriveVelocity(), null);

				builder.addDoubleProperty("Back Left Angle", () -> m_modules[2].getRotation().getRadians(), null);
				builder.addDoubleProperty("Back Left Velocity", () -> m_modules[2].getDriveVelocity(), null);

				builder.addDoubleProperty("Back Right Angle", () -> m_modules[3].getRotation().getRadians(), null);
				builder.addDoubleProperty("Back Right Velocity", () -> m_modules[3].getDriveVelocity(), null);

				builder.addDoubleProperty("Robot Angle", () -> getRobotRotation().getRadians(), null);
			}
		});
	}

	// Public Methods //

	@Override
	public void periodic() {
		Tracer.start("SwervePeriodic");

		odometryLock.lock();
		// Update modules and gyro
		m_gyro.periodic();
		for (SwerveModule module : m_modules)
			module.periodic();
		odometryLock.unlock();

		if (DriverStation.isDisabled()) for (SwerveModule module : m_modules)
			module.stop();

		// Log chassis current draw
		double netDraw = 0;
		for (SwerveModule module : m_modules)
			netDraw += Math.abs(module.getInputs().driveSupplyCurrentAmps)
					+ Math.abs(module.getInputs().turnSupplyCurrentAmps);
		SmartDashboard.putNumber("Chassis Current Draw (A)", roundDigits(netDraw, 1));

		var speeds = getChassisSpeedsFieldRelative();
		SmartDashboard.putNumber("Chassis Speed (m ∕ s)",
				roundDigits(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), 2));

		// SmartDashboard.putNumber("Chassis Rotation (deg)", roundDigits(getHeading(),
		// 1));

		// Calculate acceleration
		m_accelField = getChassisSpeedsFieldRelative().minus(m_lastChassisSpeeds)
				.div(Timer.getFPGATimestamp() - (m_lastTick == -1 ? Timer.getFPGATimestamp() : m_lastTick));
		m_lastChassisSpeeds = getChassisSpeedsFieldRelative();
		m_lastTick = Timer.getFPGATimestamp();

		// Update the odometry and pose estimate

		// Since all odometry states are sampled at the same timestamps, we can just use any module's
		// timestamps list
		double[] gyroTimestamps = m_gyro.getInputs().odometryYawTimestamps;
		double[] odomTimestamps = m_modules[0].getInputs().odometryTimestamps;
		for (int i = 0; i < odomTimestamps.length; i++) {
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
			for (int moduleIdx = 0; moduleIdx < 4; moduleIdx++)
				modulePositions[moduleIdx] = m_modules[moduleIdx].getOdometryPositions()[i];

			m_poseEstimator.updateWithTime(odomTimestamps[i],
					findNearestGyroMeasurement(odomTimestamps[i], gyroTimestamps,
							m_gyro.getInputs().odometryYawPositions),
					modulePositions); /* m_gyro.getInputs().odometryYawPositions[i], modulePositions) */
		}
		Logger.recordOutput("Drive/EstimatedPose", getFilteredPose());

		// Display vision pose on the field
		m_field.setRobotPose(getFilteredPose());

		Tracer.finish("SwervePeriodic");
	}

	/**
	 * Returns the odometry-vision estimated pose via a pose estimator.
	 *
	 * @return The estimated pose.
	 */
	public Pose2d getFilteredPose() { return m_poseEstimator.getEstimatedPosition(); }

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * <p>
	 * <b>Note</b>: This also sets the gyro yaw to match the pose's rotation.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
		m_gyro.getIO().setYaw(pose.getRotation().getDegrees());
		if (m_resetSim != null) m_resetSim.accept(pose);
	}

	/**
	 * Drives the robot with the given speeds.
	 *
	 * @param xSpeed        Speed of the robot in the x direction in m/s (forward).
	 * @param ySpeed        Speed of the robot in the y direction in m/s (sideways).
	 * @param rot           Angular rate of the robot in rad/s.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field or the
	 *                      robot.
	 */
	public void runSpeeds(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		runSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rot), fieldRelative);
	}

	/**
	 * Drives the robot with the given speeds.
	 *
	 * @param speeds        The chassis speeds to drive with.
	 * @param fieldRelative Whether the provided speeds are relative to the field or the robot.
	 */

	public void runSpeeds(ChassisSpeeds speeds, boolean fieldRelative) {
		if (fieldRelative) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRobotRotation());

		speeds = correctSkew(speeds);
		speeds = limitAcceleration(speeds);
		speeds = ChassisSpeeds.discretize(speeds, Constants.kLoopPeriodSeconds);
		var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
		for (int i = 0; i < 4; i++)
			m_modules[i].setDesiredState(swerveModuleStates[i]);
	}

	/**
	 * Corrects for the skew that worsens as angular velocity increases.
	 *
	 * @param robotRelativeVelocity The desired robot-relative speeds.
	 * @return The speeds of the robot after correction.
	 */
	public ChassisSpeeds correctSkew(ChassisSpeeds robotRelativeVelocity) {
		var angularVelocity = new Rotation2d(
				Math.toRadians(m_gyro.getInputs().omegaDegreesPerSecond) * kSkewCorrectionFactor);
		if (!Utils.epsilonEquals(angularVelocity.getRadians(), 0.0)) {
			ChassisSpeeds fieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity,
					getRobotRotation());
			robotRelativeVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeVelocity,
					getRobotRotation().plus(angularVelocity));
		}
		return robotRelativeVelocity;
	}

	/** Returns the average velocity of the modules in m/s. */
	public double getFeedforwardCharacterizationVelocity() {
		double output = 0.0;
		for (int i = 0; i < 4; i++)
			output += m_modules[i].getDriveVelocity() / 4.0;
		return output;
	}

	/**
	 * Locks the wheels into an X formation to prevent movement and sliding.
	 */
	public void setX() {
		Rotation2d[] rots = new Rotation2d[] { Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-45),
				Rotation2d.fromDegrees(-45), Rotation2d.fromDegrees(45), };
		for (int i = 0; i < 4; i++)
			m_modules[i].setDesiredState(new SwerveModuleState(0, rots[i]));
		kDriveKinematics.resetHeadings(rots);
	}

	/**
	 * Stops the chassis.
	 */
	public void stop() {
		runSpeeds(new ChassisSpeeds(), true);
	}

	/**
	 * Runs system characterization with the given voltage.
	 *
	 * @param output The voltage.
	 */
	public void runCharacterization(double output) {
		for (SwerveModule module : m_modules)
			module.runCharacterization(output);
	}

	/**
	 * Returns a command to run a quasistatic test.
	 *
	 * @param direction The direction of the quasistatic test.
	 * @return The command to run the quasistatic test.
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(m_sysId.quasistatic(direction));
	}

	/**
	 * Returns a command to run a dynamic test.
	 *
	 * @param direction The direction of the dynamic test.
	 * @return The command to run the dynamic test.
	 */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(m_sysId.dynamic(direction));
	}

	/**
	 * Sets the swerve module states. Note that this doesn't use skew compensation and acceleration
	 * limits.
	 *
	 * @param desiredStates The desired swerve module states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
		for (int i = 0; i < 4; i++)
			m_modules[i].setDesiredState(desiredStates[i]);
	}

	/**
	 * Resets the drive encoders to currently read a position of 0.
	 */
	public void resetEncoders() {
		for (int i = 0; i < 4; i++)
			m_modules[i].resetEncoders();
	}

	/**
	 * Zeroes the heading of the robot. The gyro reads 0.
	 */
	public void zeroHeading() {
		m_gyro.getIO().setYaw(0);
		m_poseEstimator.resetRotation(Rotation2d.kZero);
	}

	/**
	 * Zeroes the heading of the robot, relative to the field blue alliance-relative coordinates.
	 * If blue alliance, then sets the yaw to 0, otherwise sets to 180. The converse is true if
	 * {@link SwerveConstants#kInvertGyroZero} is true.
	 */
	public void zeroHeadingFieldRelative() {
		double angle = AllianceUtil.isRedAlliance() ? 180 : 0;
		if (SwerveConstants.kInvertGyroZero) angle = (angle + 180) % 360;
		m_gyro.getIO().setYaw(angle);
		m_poseEstimator.resetRotation(Rotation2d.fromDegrees(angle));
	}

	/**
	 * Returns the current robot-relative chassis speeds.
	 *
	 * @return The chassis speeds in robot-relative coordinates.
	 */
	public ChassisSpeeds getChassisSpeedsRobotRelative() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(m_modules[0].getState(), m_modules[1].getState(),
				m_modules[2].getState(), m_modules[3].getState());
	}

	/**
	 * Returns the current field-relative chassis speeds.
	 *
	 * @return The chassis speeds in field-relative coordinates.
	 */
	@AutoLogOutput(key = "Drive/ChassisSpeeds_FieldRelative")
	public ChassisSpeeds getChassisSpeedsFieldRelative() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeedsRobotRelative(), getRobotRotation());
	}

	/**
	 * Returns the field-relative chassis acceleration, represented by a ChassisSpeeds object.
	 *
	 * @return The chassis acceleration.
	 */
	@AutoLogOutput(key = "Drive/ChassisAcceleration_FieldRelative")
	public ChassisSpeeds getAccelerationFieldRelative() { return m_accelField; }

	/**
	 * Returns the heading of the robot.
	 *
	 * @return The robot's heading in degrees, from -180 to 180.
	 */
	public double getHeading() { return MathUtil.inputModulus(getRobotRotation().getDegrees(), -180, 180); }

	/**
	 * Returns the heading of the robot.
	 *
	 * @return The robot's heading in degrees, from 0 to 360.
	 */
	public double getHeading0to360() { return MathUtil.inputModulus(getRobotRotation().getDegrees(), 0, 360); }

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second.
	 */
	public double getTurnRate() { return m_gyro.getInputs().omegaDegreesPerSecond; }

	/**
	 * Returns the vision subsystem.
	 *
	 * @return The vision subsystem.
	 */
	public LibVisionSubsystem getVisionSubsystem() { return m_vision; }

	/**
	 * Sets whether to register the vision measurements to the pose estimator.
	 *
	 * @param register True to register, false otherwise.
	 */
	public void setRegisterVisionMeasurements(boolean register) { m_registerVision = register; }

	/**
	 * Returns whether to register vision measurements.
	 *
	 * @return Whether to register vision measurements.
	 */
	public boolean isRegisterVisionMeasurements() { return m_registerVision; }

	/**
	 * Returns the gyroscope.
	 *
	 * @return The gyroscope.
	 */
	public Gyro getGyro() { return m_gyro; }

	/**
	 * Returns the current module states.
	 *
	 * @return The module states.
	 */
	@AutoLogOutput(key = "Drive/ModuleStates")
	public SwerveModuleState[] getStates() {
		return new SwerveModuleState[] { m_modules[0].getState(), m_modules[1].getState(), m_modules[2].getState(),
				m_modules[3].getState() };
	}

	/**
	 * Returns the current module positions.
	 *
	 * @return The module states.
	 */
	@AutoLogOutput(key = "Drive/ModulePositions")
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] { m_modules[0].getOdometryPosition(), m_modules[1].getOdometryPosition(),
				m_modules[2].getOdometryPosition(), m_modules[3].getOdometryPosition() };
	}

	/**
	 * Sets the chassis to drive in open-loop mode.
	 *
	 * @param voltsDrive The voltage to apply to the drive motors.
	 * @param voltsTurn  The voltage to apply to the steer motors.
	 */
	public void runOpenLoop(double voltsDrive, double voltsTurn) {
		for (int i = 0; i < 4; i++)
			m_modules[i].runOpenLoop(voltsDrive, voltsTurn);
	}

	/**
	 * Sets whether slow mode is enabled for precise movement. Only effective for joystick control.
	 *
	 * @param isSlowMode True to enable slow mode, false to disable.
	 */
	public void setSlowMode(boolean isSlowMode) { m_isSlowMode = isSlowMode; }

	/**
	 * Returns whether slow mode is enabled.
	 *
	 * @return Whether slow mode is enabled.
	 */
	public boolean isSlowMode() { return m_isSlowMode; }

	/**
	 * Returns the robot's heading as a Rotation2d.
	 *
	 * @return The rotation.
	 */
	public Rotation2d getRobotRotation() { return Rotation2d.fromDegrees(m_gyro.getInputs().yawDegrees); }

	/**
	 * Returns the swerve pose estimator.
	 *
	 * @return The swerve pose estimator.
	 */
	public SwerveDrivePoseEstimator getPoseEstimator() { return m_poseEstimator; }

	// Private Helpers //

	private Rotation2d findNearestGyroMeasurement(double timestamp, double[] gyroTimestamps,
			Rotation2d[] gyroPositions) {
		int closestIndex = 0;
		double minDifference = Double.MAX_VALUE;
		for (int j = 0; j < gyroTimestamps.length; j++) {
			double difference = Math.abs(gyroTimestamps[j] - timestamp);
			if (difference < minDifference) { minDifference = difference; closestIndex = j; }
		}
		return gyroPositions[closestIndex];
	}

	private ChassisSpeeds limitAcceleration(ChassisSpeeds robotRelativeSpeeds) {
		return new ChassisSpeeds(m_xLimiter.calculate(robotRelativeSpeeds.vxMetersPerSecond),
				m_yLimiter.calculate(robotRelativeSpeeds.vyMetersPerSecond),
				m_rotLimiter.calculate(robotRelativeSpeeds.omegaRadiansPerSecond));
	}
}
