package com.flarerobotics.lib.control.shooter;

import static com.flarerobotics.lib.utils.Utils.EPSILON;

import com.flarerobotics.lib.container.Vector3;
import com.flarerobotics.lib.control.shooter.data.LibShooterDescriptor;
import com.flarerobotics.lib.control.shooter.data.ProjectileTarget;
import com.flarerobotics.lib.control.shooter.data.ShooterProjectile;
import com.flarerobotics.lib.control.shooter.data.ShooterProjectileType;
import com.flarerobotics.lib.control.shooter.data.ShooterState;
import com.flarerobotics.lib.control.shooter.data.ShooterState.ShooterStateStatus;
import com.flarerobotics.lib.math.PhysicsUtil;
import com.flarerobotics.lib.subsystem.vision.LibVisionSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates the optimal shooter parameters (angle, wheel speed, yaw) for hitting a target.
 */
public class ShooterCalculator {

	// Public Constants //

	/**
	 * The number of maximum iterations for dynamic motion move solve.
	 */
	public double DynamicMoveSolveIterations = 6;

	/**
	 * The RPM threshold for convergence in dynamic motion solve.
	 */
	public double DynamicMoveSolveRPMThreshold = 3;

	/**
	 * The time threshold for convergence in dynamic motion solve.
	 */
	public double DynamicMoveSolveTimeThreshold = 0.05;

	/**
	 * The maximum noise in the roll (X axis rotation) for projectile visualization. Do not
	 * increase too much due to it causing choppy rotation. Uses radians as the unit of angle.
	 */
	public double MaxRollNoiseRad = Math.toRadians(2);

	/**
	 * The list of projectile targets to use for trajectory visualization.
	 */
	public List<ProjectileTarget> ProjectileTargets = List.of();

	/**
	 * The default tolerance for the projectile target in meters.
	 */
	public double DefaultProjectileTolerance = 0.14;

	/**
	 * The maximum time-of-flight to simulate for a given object.
	 */
	public double TrajectoryMaxTime = 6;

	/**
	 * A constant velocity to apply when the velocity is too low to avoid the projectile getting
	 * stuck.
	 */
	public double VelocityEpsilon = 0.1;

	/**
	 * The threshold to apply the velocity compensation.
	 */
	public double VelocityEpsilonThreshold = 0.1;

	// Private Constants && Variables //

	private static final double kRPMRadCF = 2 * Math.PI / 60;

	private final LibShooterDescriptor m_descriptor;
	private boolean m_useSpinup;

	private double m_cachedMOI = -1;

	/**
	 * Constructs a new ShooterCalculator with the given descriptor.
	 * <p>
	 * <b>Note: The descriptors must be set via {@link LibShooterDescriptor.Builder}'s dataSet
	 * methods. Otherwise throws an exception.</b>
	 *
	 * @param descriptor The shooter subsystem descriptor containing physical parameters and
	 *                   interpolators.
	 * @throws IllegalArgumentException If the interpolators are not defined.
	 */
	public ShooterCalculator(LibShooterDescriptor descriptor) {
		if (descriptor.getAngleInterpolator() == null || descriptor.getRPMInterpolator() == null)
			throw new IllegalArgumentException("null interpolators are not allowed");
		m_descriptor = descriptor;
		m_useSpinup = descriptor.getWindupTimes() != null;
	}

	// Public Methods //

	/**
	 * Calculates a shooter state based on the provided parameters, accounting for dynamic motion.
	 * Put zero for the kinematics parameters to calculate static state. Make sure to call this
	 * method right up until before the shot to constantly update the speeds, otherwise large
	 * under/overshoots may occur.
	 *
	 * <p>
	 * Note: The method may be slightly performance intensive if called periodically.
	 * <p>
	 * <b>Note: The turret yaw assumes left is positive, as per WPILib coordinate systems. Negate
	 * the angle if your turret uses right as positive.</b>
	 * <p>
	 * <b>Note: This method may fail or give false data if there are obstacles in the way.</b>
	 * <p>
	 * <b>Note: Make sure to do a warm-up call to the method! The first call will be slower.</b>
	 *
	 * @param robotPose  The field-relative current robot pose.
	 * @param targetPose The field-relative target pose.
	 * @param currentRPM The current angular velocity of the shooter in RPM.
	 * @param projectile The projectile instance being shot.
	 * @param speeds     The field-relative chassis speeds of the robot.
	 * @param aXMeters   The field-relative acceleration along the X axis (m/s^2).
	 * @param aYMeters   The field-relative acceleration along the Y axis (m/s^2).
	 * @param aThetaRads The field-relative angular acceleration (rad/s^2).
	 * @return The computed shooter state.
	 */
	public ShooterState calculateDynamic(Pose3d robotPose, Pose3d targetPose, double currentRPM,
			ShooterProjectileType projectile, ChassisSpeeds speeds, double aXMeters, double aYMeters,
			double aThetaRads) {
		Pose3d dynamicPose = robotPose;
		ShooterState state = calculate_staticGeometry(dynamicPose, targetPose, projectile, speeds.vxMetersPerSecond,
				speeds.vyMetersPerSecond);

		double rpm = state.getWheelRPM();
		double t = (m_useSpinup
				? Math.abs(m_descriptor.getWindupTimes().get(rpm) - m_descriptor.getWindupTimes().get(currentRPM))
						* m_descriptor.getNominalVoltage() / getBatteryVoltage()
				: 0) + m_descriptor.getHopperDelay();

		// Converge via iteration
		int iteration = 0;
		while (iteration++ < DynamicMoveSolveIterations) {
			// Compute new pose
			double x = robotPose.getX() + speeds.vxMetersPerSecond * t + 0.5 * aXMeters * t * t;
			double y = robotPose.getY() + speeds.vyMetersPerSecond * t + 0.5 * aYMeters * t * t;
			double theta = MathUtil.angleModulus(robotPose.getRotation().toRotation2d().getRadians()
					+ speeds.omegaRadiansPerSecond * t + 0.5 * aThetaRads * t * t); // Wrap angle [-180, 180]

			// Limits
			x = MathUtil.clamp(x, 0, LibVisionSubsystem.kLayout.getFieldLength());
			y = MathUtil.clamp(y, 0, LibVisionSubsystem.kLayout.getFieldWidth());

			dynamicPose = new Pose3d(x, y, robotPose.getZ(), new Rotation3d(Rotation2d.fromRadians(theta)));
			state = calculate_staticGeometry(dynamicPose, targetPose, projectile,
					speeds.vxMetersPerSecond + aXMeters * t, speeds.vyMetersPerSecond + aYMeters * t);
			// if (!state.isPossible()) { return new ShooterState(0, 0, 0, false, 0); }

			double lastRPM = rpm;
			double lastT = t;
			rpm = state.getWheelRPM();
			t = (m_useSpinup
					? Math.abs(m_descriptor.getWindupTimes().get(rpm) - m_descriptor.getWindupTimes().get(currentRPM))
							* m_descriptor.getNominalVoltage() / getBatteryVoltage()
					: 0) + m_descriptor.getHopperDelay();

			// Convergence check
			if (Math.abs(rpm - lastRPM) < DynamicMoveSolveRPMThreshold
					&& Math.abs(t - lastT) < DynamicMoveSolveTimeThreshold) {
				return state;
			}
		}

		DriverStation.reportWarning("WARNING: ShooterCalculator::calculateDynamic, state may not have converged", true);
		return state;
	}

	/**
	 * Updates the projectile trajectory visualizer using AdvantageScope. Logs the values to the
	 * given logger key. If performance intensive, move to a Notifier thread.
	 *
	 * @param loggerKey The key to put the projectiles' positions into.
	 * @param list      The list of projectiles to simulate.
	 * @return The updated list, removing all ended projectiles.
	 */
	public List<ShooterProjectile> updateVisualizer(String loggerKey, List<ShooterProjectile> list) {
		List<Pose3d> poses = new ArrayList<>();
		List<ShooterProjectile> newProjectiles = new ArrayList<>();
		for (ShooterProjectile projectile : list) {
			double now = Timer.getFPGATimestamp();

			// Timer not initialized, first run
			if (projectile.lastUpdate == -1) { projectile.lastUpdate = now; projectile.shotAt = now; }
			// No physics updates for frozen projectiles
			if (projectile.isFrozen) { newProjectiles.add(projectile); poses.add(projectile.pose); continue; }

			double dt = now - projectile.lastUpdate;
			// Compute new state
			// Linear approximation (fine at high freq updates): dx = v * t + 1/2 at^2
			Vector3 dx = projectile.velocity.scale(dt).add(projectile.acceleration.scale(0.5 * dt * dt));
			Pose3d lastPose = projectile.pose; // Save pose
			projectile.pose = new Pose3d(projectile.pose.getX() + dx.x, projectile.pose.getY() + dx.y,
					projectile.pose.getZ() + dx.z, rotationFromVelocity(projectile.velocity));
			projectile.velocity = projectile.velocity.add(projectile.acceleration.scale(dt));

			// Check target hits
			boolean destroy = false;
			for (ProjectileTarget target : ProjectileTargets) {
				// We use 2 poses to check for hit, for accuracy purposes
				boolean hit = target.checkHit(lastPose, projectile.pose);
				if (hit && !target.hideProjectileOnHit) {
					projectile.isFrozen = true;
					break;
				} else if (hit && target.hideProjectileOnHit) { destroy = true; break; }
			}
			// Just skip if set to remove
			if (destroy) continue;
			// End condition
			if (projectile.pose.getZ() >= 0 && now - projectile.shotAt <= TrajectoryMaxTime) {
				poses.add(projectile.pose);
				newProjectiles.add(projectile);
			}
			// Update new state
			Vector3 v = projectile.velocity;
			double k = projectile.getType().getDragForceCoeff();
			// -F_net = 1/2 * Cd * rho * A * v * |v| + mg -> (grav + drag)
			Vector3 force = v.scale(k * v.norm())
					.add(new Vector3(0, 0, PhysicsUtil.kG * projectile.getType().getMass()));
			// a = F/m
			Vector3 accel_new = force.scale(-1 / projectile.getType().getMass());
			projectile.acceleration = accel_new;
			projectile.lastUpdate = now;
		}

		Logger.recordOutput(loggerKey, poses.toArray(new Pose3d[poses.size()]));
		return newProjectiles;
	}

	// Private Helper Methods //

	private ShooterState calculate_staticGeometry(Pose3d robotPose, Pose3d targetPose, ShooterProjectileType proj,
			double chassisVx, double chassisVy) {
		// Calculate deltas relative to the pivot
		var rel = targetPose.relativeTo(robotPose.plus(new Transform3d(m_descriptor.getPivotOffset())))
				.getTranslation();
		double dx = rel.getX();
		double dy = rel.getY();
		double dz = rel.getZ();

		// Static geometry
		double horizontalDist = Math.hypot(dx, dy);
		double hoodAngleDeg = m_descriptor.getAngleInterpolator().interpolate(horizontalDist, dz);
		double hoodAngleRad = Math.toRadians(hoodAngleDeg);

		// Trig values
		double cosP = Math.cos(hoodAngleRad);
		double sinP = Math.sin(hoodAngleRad);

		// As if stationary exit speeds
		double vStationary_rr = computeExitVelocityFromRPM(
				m_descriptor.getRPMInterpolator().interpolate(horizontalDist, dz), proj);
		double vHorizStat_rr = vStationary_rr * cosP;

		ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(chassisVx, chassisVy, 0,
				robotPose.getRotation().toRotation2d());

		double ux, uy; // The aim vectors
		try {
			// Solve for intercept time and direction
			double[] sol = intercept2D(dx, dy, robotRelative.vxMetersPerSecond, robotRelative.vyMetersPerSecond,
					vHorizStat_rr);
			// t = sol[0];
			ux = sol[1];
			uy = sol[2];
		} catch (IllegalArgumentException e) {
			// No intercept possible
			return new ShooterState(hoodAngleDeg, 0, 0, 0, false, 0, new Vector3(0, 0, 0), robotPose,
					ShooterStateStatus.kTargetUnreachable, "target unreachable: " + e.toString());
		}

		// Calculate yaw to hit the target
		// The angle is the same as atan2(dy, dx) since we are using the unit vector
		double yawRad = Math.atan2(uy, ux);
		// Convert to degrees, no need for relativity as uy,ux are already relative
		double yawDeg = Math.toDegrees(yawRad);

		if (MathUtil.inputModulus(m_descriptor.getPivotOffset().getRotation().getRadians(), 0, 2 * Math.PI) > EPSILON) {
			// We need to account for the angular offset of the pivot
			Rotation2d angle = new Rotation2d(yawRad).minus(m_descriptor.getPivotOffset().getRotation());
			yawRad = angle.getRadians();
			yawDeg = angle.getDegrees();
			uy = angle.getSin();
			ux = angle.getCos();

			// Re-normalize to avoid numerical issues
			double norm = Math.hypot(ux, uy);
			ux /= norm;
			uy /= norm;
		}

		// Projecting the velocities along the shot direction
		// This is simply V_a = vx * ux + vy * uy
		double v_along = robotRelative.vxMetersPerSecond * ux + robotRelative.vyMetersPerSecond * uy;

		// Robot-relative speeds:
		// v_rr = v_fr - v_a, since the velocity along the shot axis will compensate for the lowered
		// RPM and exit velocity. We subtract because the robot velocity along the shot axis
		// contributes, so we need to shoot slower
		double vHorizontalDynamic = vHorizStat_rr - v_along;
		double vVertical = vStationary_rr * sinP;

		if (vHorizontalDynamic < EPSILON) return new ShooterState(hoodAngleDeg, 0, 0, 0, false, 0, new Vector3(0, 0, 0),
				robotPose, ShooterStateStatus.kNumericalFailure, "target unreachable: illegal velocity");

		double vDynamic_rr = Math.hypot(vVertical, vHorizontalDynamic);

		// To avoid projectile getting stuck just in case, apply v_eps
		if (vDynamic_rr <= VelocityEpsilonThreshold) vDynamic_rr += VelocityEpsilon;

		double rpmFinal = computeRPMFromExitVelocity(vDynamic_rr, proj);

		boolean isPossible = true;
		// Check for RPM limits and hardware limits
		if (rpmFinal > m_descriptor.getMaxRPM() || hoodAngleDeg > m_descriptor.getMaxAngle()
				|| hoodAngleDeg < m_descriptor.getMinAngle())
			isPossible = false;

		// Since we've decoupled the horizontal and vertical velocity, we must re-adjust then pitch
		// A = arctan(vz / vx)
		hoodAngleRad = Math.atan(vVertical / vHorizontalDynamic);
		hoodAngleDeg = Math.toDegrees(hoodAngleRad);
		// Recompute trig values
		sinP = Math.sin(hoodAngleRad);
		cosP = Math.cos(hoodAngleRad);

		// Convert from robot-relative exit speeds to field-relative
		double e_vh = vDynamic_rr * cosP;
		double e_vz = vDynamic_rr * sinP;

		// Projected speeds
		double e_vx_robot = e_vh * ux;
		double e_vy_robot = e_vh * uy;
		double e_vz_robot = e_vz;

		// Trig values of the yaw
		double cosA = Math.cos(robotPose.getRotation().toRotation2d().getRadians());
		double sinA = Math.sin(robotPose.getRotation().toRotation2d().getRadians());

		// Rotate to match field-relative
		double e_vx_fr = cosA * e_vx_robot - sinA * e_vy_robot;
		double e_vy_fr = sinA * e_vx_robot + cosA * e_vy_robot;
		double e_vz_fr = e_vz_robot;

		// Account for chassis speeds to get absolute field-relative speeds
		double e_vx_field_abs = e_vx_fr + chassisVx;
		double e_vy_field_abs = e_vy_fr + chassisVy;
		double e_vz_field_abs = e_vz_fr;

		// Total exit speed
		double vExitField = Math.sqrt(
				e_vx_field_abs * e_vx_field_abs + e_vy_field_abs * e_vy_field_abs + e_vz_field_abs * e_vz_field_abs);

		// Optimize velocity && angle to match exit vector
		Vector3 desiredExitVelocity = new Vector3(e_vx_field_abs, e_vy_field_abs, e_vz_field_abs);

		// Recompute hood angle to match the exit vector (just to make sure)
		// A = arctan(vz / vx)
		hoodAngleRad = Math.atan(desiredExitVelocity.z / Math
				.sqrt(desiredExitVelocity.x * desiredExitVelocity.x + desiredExitVelocity.y * desiredExitVelocity.y));
		hoodAngleDeg = Math.toDegrees(hoodAngleRad);
		// Recompute trig values
		sinP = Math.sin(hoodAngleRad);
		cosP = Math.cos(hoodAngleRad);

		// Compute exit pose (end of barrel)
		// Pivot central offset
		Transform3d pivotTransform = new Transform3d(m_descriptor.getPivotOffset());
		// Pivot height offset
		Transform3d heightTransform = new Transform3d(new Translation3d(0.0, 0.0, m_descriptor.getPivotHeight()),
				Rotation3d.kZero);
		// Hood transform via vector decomposition
		Transform3d hoodTransform = new Transform3d(
				new Translation3d(-m_descriptor.getBarrelLength() * cosP * cosA,
						m_descriptor.getBarrelLength() * cosP * sinA, m_descriptor.getBarrelLength() * sinP),
				new Rotation3d(0.0, -hoodAngleRad, yawRad));

		// Final exit pose via all transforms
		Pose3d exitPose = robotPose.transformBy(pivotTransform).transformBy(heightTransform).transformBy(hoodTransform);

		if (vHorizontalDynamic >= EPSILON) {
			// No-drag apex approximation check to make sure the target is reachable
			double maxHeight = Math.pow(vExitField * sinP, 2) / (2 * Math.abs(PhysicsUtil.kG));
			double requiredHeight = dz + 0.5 * Math.abs(PhysicsUtil.kG) * (horizontalDist / vHorizontalDynamic)
					* (horizontalDist / vHorizontalDynamic) - m_descriptor.getPivotHeight();

			if (requiredHeight - maxHeight > m_descriptor.getValidityCheckHeightTol()) isPossible = false;
		} else {
			// Vertical shot, just check if we can reach the height
			double maxHeight = Math.pow(vExitField, 2) / (2 * Math.abs(PhysicsUtil.kG));
			double requiredHeight = (dz + m_descriptor.getPivotHeight()) - exitPose.getZ();

			if (requiredHeight - maxHeight > m_descriptor.getValidityCheckHeightTol()) isPossible = false;
		}

		// Re-check for RPM limits and hardware limits
		if (rpmFinal > m_descriptor.getMaxRPM() || hoodAngleDeg > m_descriptor.getMaxAngle()
				|| hoodAngleDeg < m_descriptor.getMinAngle())
			isPossible = false;

		// Return final state
		return new ShooterState(hoodAngleDeg, yawDeg, exitPose.getRotation().toRotation2d().getDegrees(), rpmFinal,
				isPossible, desiredExitVelocity.norm(), desiredExitVelocity, exitPose,
				(isPossible ? ShooterStateStatus.kOk : ShooterStateStatus.kTargetUnreachable),
				(isPossible ? "ok" : "target unreachable"));
	}

	// Returns the battery voltage
	private static double getBatteryVoltage() {
		if (Robot.isSimulation()) return RoboRioSim.getVInVoltage();
		else return RobotController.getBatteryVoltage();
	}

	// There is a linear relationship between w and the exit velocity:
	// v_ext = e*w*r*(I_wheel / (I_wheel + I_projectile))
	private double computeRPMFromExitVelocity(double v, ShooterProjectileType proj) {
		double moi = getTotalInertia();
		double w = v * (moi + proj.getEffectiveInertia()) / (moi * proj.getECoeff() * m_descriptor.getRollerRadius());

		return w / kRPMRadCF;
	}

	// Computes the exit velocity from the given RPM
	private double computeExitVelocityFromRPM(double rpm, ShooterProjectileType proj) {
		double moi = getTotalInertia();
		double w = rpm * kRPMRadCF;
		return proj.getECoeff() * w * m_descriptor.getRollerRadius() * moi / (moi + proj.getEffectiveInertia());
	}

	// Returns the total system inertia
	private double getTotalInertia() {
		if (m_cachedMOI >= 0) return m_cachedMOI;

		double totalMOI;
		if (m_descriptor.getFlywheelToShooterReduction() != 0 && m_descriptor.getFlywheelMOI() != 0) {
			totalMOI = m_descriptor.getFlywheelMOI() / Math.pow(m_descriptor.getFlywheelToShooterReduction(), 2)
					+ m_descriptor.getRollerMOI();
		} else {
			totalMOI = m_descriptor.getRollerMOI();
		}
		m_cachedMOI = totalMOI;

		return totalMOI;
	}

	// Makes the projectile look in the same direction as their velocity
	// Also applies random roll shifting
	private Rotation3d rotationFromVelocity(Vector3 v) {
		if (v.norm() < 1e-6) {
			// no movement, identity rotation with a little roll noise
			double rollNoise = randomRoll();
			return new Rotation3d(rollNoise, 0.0, 0.0);
		}
		double yaw = Math.atan2(v.y, v.x);
		double pitch = -Math.atan2(v.z, Math.hypot(v.x, v.y));
		double roll = randomRoll();

		// Rotation3d(roll, pitch, yaw)
		return new Rotation3d(roll, pitch, yaw);
	}

	// Creates a random X (roll) rotation
	private double randomRoll() {
		return Math.random() * (2 * MaxRollNoiseRad) - MaxRollNoiseRad;
	}

	// Equations taken and modified from:
	// https://stackoverflow.com/questions/17204513/how-to-find-the-interception-coordinates-of-a-moving-target-in-3d-space/22117046#22117046
	//
	// Solves (dx - v_x t)^2 + (dy - v_y t)^2 = (s t)^2 for the smallest positive t, and return
	// [t, ux, uy].
	private static double[] intercept2D(double dx, double dy, double vx, double vy, double s) {
		// Solve parameters
		double a = vx * vx + vy * vy - s * s;
		double b = -2 * (dx * vx + dy * vy);
		double c = dx * dx + dy * dy;

		double t = Double.POSITIVE_INFINITY;
		if (Math.abs(a) < EPSILON) {
			// Linear fallback: bt + c = 0
			if (Math.abs(b) < EPSILON) throw new IllegalArgumentException("no intercept possible (invalid)");
			t = -c / b;
			if (t <= 0) throw new IllegalArgumentException("no positive intercept time");
		} else {
			// Discriminant
			double disc = b * b - 4 * a * c;
			if (disc < 0) throw new IllegalArgumentException("no intercept possible");
			// Solve quadratic
			double sqrtD = Math.sqrt(disc);
			// double t1 = (-b + sqrtD) / (2 * a);
			// double t2 = (-b - sqrtD) / (2 * a);

			// Use numerically stable quadratic solver
			double q = (b >= 0.0) ? -0.5 * (b + sqrtD) : -0.5 * (b - sqrtD);
			double t1 = q / a;
			double t2 = c / q;
			// Pick smallest positive
			if (t1 > 0) t = Math.min(t, t1);
			if (t2 > 0) t = Math.min(t, t2);
			if (!Double.isFinite(t)) throw new IllegalArgumentException("no positive intercept time");
		}

		// Validate s*t !~= 0
		double st = s * t;
		if (Math.abs(st) < EPSILON) throw new IllegalArgumentException("no intercept possible (s*t ~= 0)");

		// Direction vector <ux, uy>
		double ux = (dx - vx * t) / st;
		double uy = (dy - vy * t) / st;
		// Normalize (just in case of numerical drift)
		double norm = Math.hypot(ux, uy);
		return new double[] { t, ux / norm, uy / norm };
	}
}
