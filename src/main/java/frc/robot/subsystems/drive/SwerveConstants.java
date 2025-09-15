package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.PhysicalParameters;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * The constants required for the holonomic drive chassis.
 */
public final class SwerveConstants {
	// SysId routine time limits
	public static final double kDynamicTestTimeLimit = 3.0;
	public static final double kQuasistaticTestTimeLimit = 8.0;

	// Frequency of the high-frequency odometry thread
	// Keep in mind this follows the default 250Hz if in sim
	public static final double kOdometryFrequencyHz = 100;

	// Whether the gyro angle should be inverted when zeroGyroFieldRelative is called
	public static final boolean kInvertGyroZero = false;

	/** Class holding the drive constants. */
	public static final class DriveConstants {
		// Maximum allowed speeds (not the hardware max)
		public static final double kMaxSpeedMetersPerSecond = 4.8; // m/s
		public static final double kMaxAngularSpeed = 2 * Math.PI; // rad/s

		public static final double kMaxLinearAcceleration = Double.POSITIVE_INFINITY; // m/s^2
		public static final double kMaxAngularAcceleration = Double.POSITIVE_INFINITY; // rad/s^2

		// Haven't measured this, just a guess
		public static final double kHardwareMaxSpeed = 4.8;

		public static final double kSlowModeTranslationalMultiplier = 0.2;
		public static final double kSlowModeRotationalMultiplier = 0.35;

		// Chassis configuration
		// Distance between the centers of the right and left wheels on robot
		public static final double kTrackWidth = 0.68; // meters
		// Distance between the centers of the front and back wheels on robot
		public static final double kWheelBase = 0.68; // meters
		// Instantiate the kinematics
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		public static final double kDrivebaseRadius = Math.hypot(kTrackWidth / 2, kWheelBase / 2);

		// Angular offsets of the modules relative to the chassis in radians
		/*
		 * public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2; public static
		 * final double kFrontRightChassisAngularOffset = 0; public static final double
		 * kBackLeftChassisAngularOffset = Math.PI; public static final double
		 * kBackRightChassisAngularOffset = Math.PI / 2;
		 */
		// TODO test angular offsets, taken from 2025 code
		public static final double kFrontLeftChassisAngularOffset = Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = -Math.PI / 2;
		public static final double kBackLeftChassisAngularOffset = 0;
		public static final double kBackRightChassisAngularOffset = -Math.PI / 2;

		public static final double kBrownoutVoltage = 6.8;
		static {
			// Only effective on the roboRIO 2
			RobotController.setBrownoutVoltage(kBrownoutVoltage);
		}

		// SPARK MAX CAN IDs
		public static final int kFrontLeftDrivingCanId = 9;
		public static final int kRearLeftDrivingCanId = 3;
		public static final int kFrontRightDrivingCanId = 4;
		public static final int kRearRightDrivingCanId = 8;

		public static final int kFrontLeftTurningCanId = 19;
		public static final int kRearLeftTurningCanId = 13;
		public static final int kFrontRightTurningCanId = 14;
		public static final int kRearRightTurningCanId = 18;

		public static final int kPigeon2CanId = 7;

		public static final boolean kCosineCompensateReal = true;
		public static final boolean kCosineCompensateSim = false;

		// Factor to correct for skews in the swerve modules while steering and driving
		// * Start with 0 to figure out the default skew direction when rotating in a specific
		// direction
		// Ideally should be in the range -0.15 to 0.15, start tuning with 0.1
		// * If skew gets worse, negate the value
		// * If the skew changes direction while rotating in the same direction, make the value closer
		// to 0
		// * For precision, use the identification command
		public static final double kSkewCorrectionFactor = 0.06508;

		// Apply a velocity deadband to prevent jittering
		public static final double kJitterVelocityDeadbandMps = 2E-3;
	}

	/** Class holding the MAXSwerve modules' constants. */
	public static final class ModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T,
		// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
		// more teeth will result in a robot that drives faster).
		public static final int kDrivingMotorPinionTeeth = 14; // TODO check

		// Calculations required for driving motor conversion factors and feed forward
		public static final double kDrivingMotorFreeSpeedRps = DriveMotorConstants.kFreeSpeedRpm / 60;
		// The wheel diameter of the MAXSwerve module is 3in, which is 0.0762m
		public static final double kWheelDiameterMeters = Units.inchesToMeters(3); // meters
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
		// teeth on the bevel pinion
		public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
		public static final double kDriveWheelFreeSpeed = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
				/ kDrivingMotorReduction;

		public static final double kTurningReduction = 9424.0 / 203.0;

		// Maximum motor temperatures, in Celsius
		public static final double kDriveMotorMaxTemperature = 80;
		public static final double kTurnMotorMaxTemperature = 80;

		// Whether to forcefully stop the robot when the motors overheat
		public static final boolean kEmergencyStopOnOverheat = true;
	}

	/** Class holding the constants for the autonomous period. */
	public static final class AutoConstants {
		/*
		 * public static final double kPathDriveP = 25; // Just p = 5 works. public static final double
		 * kPathDriveI = 0.5; public static final double kPathDriveD = 2;
		 */
		// Autonomous path gains
		public static final double kPathDriveP = 4;
		public static final double kPathDriveD = 0;

		public static final double kPathTurnP = 6;
		public static final double kPathTurnD = 0;

		public static final PathConstraints kPathConstraints = new PathConstraints(4.5, 4, Math.toRadians(540),
				Math.toRadians(720));

		// Whether to apply the feedforwards provided by PathPlanner
		public static final boolean kUseAutonomousFeedforward = false;

		// The alliance to invert the paths at
		public static final Alliance kInvertAutonomousPathAlliance = Alliance.Red;
	}

	/** Class holding the module configurations. */
	public static final class ModuleConfigs {
		public static final SparkMaxConfig kDrivingConfig = new SparkMaxConfig();
		public static final SparkMaxConfig kTurningConfig = new SparkMaxConfig();

		// The gearbox used on each MAXSwerve module (ignoring reductions)
		public static final DCMotor kPerModuleDriveGearbox = DCMotor.getFalcon500(1);
		public static final DCMotor kPerModuleTurnGearbox = DCMotor.getNeo550(1);

		// TODO
		public static final double kDriveP = 0.04, kDriveI = 0, kDriveD = 0;
		public static final double kTurnP = 1, kTurnI = 0, kTurnD = 0;

		public static final double kDrivePSim = 10, kDriveISim = 0, kDriveDSim = 0;
		public static final double kTurnPSim = 8, kTurnISim = 0, kTurnDSim = 0;

		public static final double kDriveKs = 0.03457, kDriveKv = 2.55804;
		public static final double kDriveKsSim = 0.03457, kDriveKvSim = 2.55804;

		public static final int kDriveMotorCurrentLimit = 50;
		public static final int kTurnMotorCurrentLimit = 20;

		static {
			// Use module constants to calculate conversion factors and feed forward gain
			double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
					/ ModuleConstants.kDrivingMotorReduction;
			double turningFactor = 2 * Math.PI;
			// double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeed;

			int odometryUpdatePeriod = (int) (1000.0 / kOdometryFrequencyHz);
			// Driving configuration
			kDrivingConfig.idleMode(IdleMode.kBrake)
					.smartCurrentLimit(kDriveMotorCurrentLimit)
					.voltageCompensation(PhysicalParameters.kNominalVoltage);
			// Update signals at the same rate as the odometry update period
			kDrivingConfig.signals.primaryEncoderPositionAlwaysOn(true)
					.primaryEncoderPositionPeriodMs(odometryUpdatePeriod)
					.primaryEncoderVelocityAlwaysOn(true)
					.primaryEncoderVelocityPeriodMs(odometryUpdatePeriod);
			kDrivingConfig.encoder.positionConversionFactor(drivingFactor) // meters
					.velocityConversionFactor(drivingFactor / 60.0); // m/s
			kDrivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
					.pid(kDriveP, kDriveI, kDriveD) // TODO
					// No feedforward as we have it in the SwerveModuleIOs
					// .velocityFF(drivingVelocityFeedForward)
					.outputRange(-1, 1);

			// Turning configuration
			kTurningConfig.idleMode(IdleMode.kBrake)
					.smartCurrentLimit(kTurnMotorCurrentLimit)
					.voltageCompensation(PhysicalParameters.kNominalVoltage);
			// Update signals at the same rate as the odometry update period
			kTurningConfig.signals.absoluteEncoderPositionAlwaysOn(true)
					.absoluteEncoderPositionPeriodMs(odometryUpdatePeriod)
					.absoluteEncoderVelocityAlwaysOn(true)
					.absoluteEncoderVelocityPeriodMs(odometryUpdatePeriod);
			kTurningConfig.absoluteEncoder
					// Invert the turning encoder, since the output shaft rotates in the opposite
					// direction of the steering motor in the MAXSwerve Module
					.inverted(true)
					.positionConversionFactor(turningFactor) // rad
					.velocityConversionFactor(turningFactor / 60.0); // rad/s
			kTurningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
					.pid(kTurnP, kTurnI, kTurnD) // TODO
					.outputRange(-1, 1)
					// Enable PID wrap around for the turning motor. This will allow the PID
					// controller to go through 0 to get to the setpoint
					.positionWrappingEnabled(true)
					.positionWrappingInputRange(0, turningFactor);
		}
	}

	/** Class holding the motor constants. */
	public static final class DriveMotorConstants {
		// Motor free speed
		public static final double kFreeSpeedRpm = Units
				.radiansPerSecondToRotationsPerMinute(ModuleConfigs.kPerModuleDriveGearbox.freeSpeedRadPerSec);
	}

	public static final class SwerveSimulationConstants {
		public static final SwerveModuleSimulationConfig kSwerveModuleSimConfig = COTS.ofMAXSwerve(
				ModuleConfigs.kPerModuleDriveGearbox, ModuleConfigs.kPerModuleTurnGearbox, PhysicalParameters.kWheelCOF,
				3);

		public static final DriveTrainSimulationConfig kSwerveSimConfig = DriveTrainSimulationConfig.Default()
				.withBumperSize(PhysicalParameters.kRobotLength, PhysicalParameters.kRobotWidth)
				.withGyro(COTS.ofPigeon2())
				.withRobotMass(PhysicalParameters.kRobotMass)
				.withTrackLengthTrackWidth(Meters.of(DriveConstants.kWheelBase), Meters.of(DriveConstants.kTrackWidth))
				.withSwerveModule(kSwerveModuleSimConfig);
	}
}
