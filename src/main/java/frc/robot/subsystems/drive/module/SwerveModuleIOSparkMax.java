package frc.robot.subsystems.drive.module;

import static com.revrobotics.spark.SparkUtils.retryUntilOk;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kBackLeftChassisAngularOffset;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kBackRightChassisAngularOffset;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kFrontLeftChassisAngularOffset;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kFrontLeftDrivingCanId;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kFrontLeftTurningCanId;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kFrontRightChassisAngularOffset;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kFrontRightDrivingCanId;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kFrontRightTurningCanId;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kRearLeftDrivingCanId;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kRearLeftTurningCanId;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kRearRightDrivingCanId;
import static frc.robot.subsystems.drive.SwerveConstants.DriveConstants.kRearRightTurningCanId;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kDriveKs;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kDriveKv;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kDrivingConfig;
import static frc.robot.subsystems.drive.SwerveConstants.ModuleConfigs.kTurningConfig;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.odometry.OdometryThread;
import frc.robot.subsystems.drive.odometry.SparkOdometryThread;
import java.util.Queue;

/** The swerve module IO implementation for the SparkMax. */
public class SwerveModuleIOSparkMax implements SwerveModuleIO {
	private static final int kMaxConfigAttempts = 5;

	private SparkMax m_driveSpark;
	private SparkMax m_turnSpark;

	private RelativeEncoder m_driveEncoder;
	private AbsoluteEncoder m_turnEncoder;

	private SparkClosedLoopController m_driveController;
	private SparkClosedLoopController m_turnController;

	private Rotation2d m_offset;

	private Queue<Double> m_odometryTimestamps;
	private Queue<Double> m_odometryDrivePositionsMeters;
	private Queue<Double> m_odometryTurnPositionsRads;

	/**
	 * Constructs a new SwerveModuleIOSparkMax.
	 *
	 * @param idx The module ID. 0 = front left, 1 = front right, 2 = back left, 3 = back right.
	 */
	public SwerveModuleIOSparkMax(int idx) {
		if (!OdometryThread.threadType.get().equals("SPARK"))
			throw new IllegalStateException("can't use SparkMAX modules with a non-spark odometry thread");

		m_driveSpark = new SparkMax(switch (idx) {
			case 0 -> kFrontLeftDrivingCanId;
			case 1 -> kFrontRightDrivingCanId;
			case 2 -> kRearLeftDrivingCanId;
			case 3 -> kRearRightDrivingCanId;
			default -> 0;
		}, MotorType.kBrushless);

		retryUntilOk(
				() -> m_driveSpark.configure(kDrivingConfig, ResetMode.kResetSafeParameters,
						PersistMode.kPersistParameters),
				kMaxConfigAttempts, "Configuring module " + idx + " - drive motor");

		m_turnSpark = new SparkMax(switch (idx) {
			case 0 -> kFrontLeftTurningCanId;
			case 1 -> kFrontRightTurningCanId;
			case 2 -> kRearLeftTurningCanId;
			case 3 -> kRearRightTurningCanId;
			default -> 0;
		}, MotorType.kBrushless);

		retryUntilOk(
				() -> m_turnSpark.configure(kTurningConfig, ResetMode.kResetSafeParameters,
						PersistMode.kPersistParameters),
				kMaxConfigAttempts, "Configuring module " + idx + " - turn motor");

		m_offset = new Rotation2d(switch (idx) {
			case 0 -> kFrontLeftChassisAngularOffset;
			case 1 -> kFrontRightChassisAngularOffset;
			case 2 -> kBackLeftChassisAngularOffset;
			case 3 -> kBackRightChassisAngularOffset;
			default -> 0;
		});

		m_driveEncoder = m_driveSpark.getEncoder();
		m_turnEncoder = m_turnSpark.getAbsoluteEncoder();

		m_driveController = m_driveSpark.getClosedLoopController();
		m_turnController = m_turnSpark.getClosedLoopController();

		SparkOdometryThread odometryThread = (SparkOdometryThread) OdometryThread.getInstance();
		m_odometryTimestamps = odometryThread.makeTimestampQueue();
		m_odometryDrivePositionsMeters = odometryThread.registerSignal(m_driveSpark, m_driveEncoder::getPosition);
		m_odometryTurnPositionsRads = odometryThread.registerSignal(m_turnSpark, m_turnEncoder::getPosition);
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		// Update drive inputs
		inputs.driveMotorConnected = m_driveSpark.getLastError() != REVLibError.kOk;
		inputs.drivePositionMeters = m_driveEncoder.getPosition();
		inputs.driveVelocityMetersPerSec = m_driveEncoder.getVelocity();
		inputs.driveAppliedVolts = m_driveSpark.getAppliedOutput() * m_driveSpark.getBusVoltage();
		inputs.driveSupplyCurrentAmps = m_driveSpark.getOutputCurrent() * m_driveSpark.getAppliedOutput();
		inputs.driveStatorCurrentAmps = m_driveSpark.getOutputCurrent();
		inputs.driveTemperatureCelsius = m_driveSpark.getMotorTemperature();

		// Update steering inputs
		inputs.driveMotorConnected = m_turnSpark.getLastError() != REVLibError.kOk;
		inputs.turnPosition = new Rotation2d(m_turnEncoder.getPosition()).minus(m_offset);
		inputs.turnVelocityRadPerSec = m_turnEncoder.getVelocity();
		inputs.turnAppliedVolts = m_turnSpark.getAppliedOutput() * m_turnSpark.getBusVoltage();
		inputs.turnSupplyCurrentAmps = m_turnSpark.getOutputCurrent() * m_turnSpark.getAppliedOutput();
		inputs.turnStatorCurrentAmps = m_turnSpark.getOutputCurrent();
		inputs.turnTemperatureCelsius = m_turnSpark.getMotorTemperature();

		// Update odometry inputs
		inputs.odometryTimestamps = m_odometryTimestamps.stream().mapToDouble((v) -> v).toArray();
		inputs.odometryDrivePositionsMeters = m_odometryDrivePositionsMeters.stream().mapToDouble((v) -> v).toArray();
		inputs.odometryTurnPositions = m_odometryTurnPositionsRads.stream()
				.map((v) -> new Rotation2d(v).minus(m_offset))
				.toArray(Rotation2d[]::new);

		m_odometryTimestamps.clear();
		m_odometryDrivePositionsMeters.clear();
		m_odometryTurnPositionsRads.clear();
	}

	@Override
	public void runDriveVelocity(double velocityMetersPerSec) {
		runDriveVelocityFF(velocityMetersPerSec, 0);
	}

	@Override
	public void runDriveVelocityFF(double velocityMetersPerSec, double ff) {
		double ffVolts = ff + kDriveKs * Math.signum(velocityMetersPerSec) + kDriveKv * velocityMetersPerSec;
		m_driveController.setReference(velocityMetersPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts,
				ArbFFUnits.kVoltage);
	}

	@Override
	public void runDriveVelocityFF(double velocityMetersPerSec, double ff, boolean onlyProvidedFF) {
		double ffVolts = ff
				+ (onlyProvidedFF ? 0 : kDriveKs * Math.signum(velocityMetersPerSec) + kDriveKv * velocityMetersPerSec);
		m_driveController.setReference(velocityMetersPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts,
				ArbFFUnits.kVoltage);
	}

	@Override
	public void runDriveOpenLoop(double output) {
		m_driveSpark.setVoltage(output);
	}

	@Override
	public void runTurnPosition(Rotation2d rotation) {
		double wrapped = MathUtil.inputModulus(rotation.plus(m_offset).getRadians(), 0, 2 * Math.PI);
		m_turnController.setReference(wrapped, ControlType.kPosition);
	}

	@Override
	public void runTurnOpenLoop(double output) {
		m_turnSpark.setVoltage(output);
	}

	@Override
	public void resetEncoders() {
		m_driveEncoder.setPosition(0);
	}
}
