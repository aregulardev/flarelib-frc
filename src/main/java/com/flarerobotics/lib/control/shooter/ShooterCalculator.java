package com.flarerobotics.lib.control.shooter;

import com.flarerobotics.lib.container.Vector3;
import com.flarerobotics.lib.control.shooter.data.LibShooterDescriptor;
import com.flarerobotics.lib.control.shooter.data.ProjectileTarget;
import com.flarerobotics.lib.control.shooter.data.ShooterProjectile;
import com.flarerobotics.lib.control.shooter.data.ShooterProjectileType;
import com.flarerobotics.lib.control.shooter.data.ShooterState;
import com.flarerobotics.lib.math.PhysicsUtil;
import com.flarerobotics.lib.subsystem.vision.LibVisionSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    public static double kDynamicMoveSolveIterations = 6;

    /**
     * The RPM threshold for convergence in dynamic motion solve.
     */
    public static double kDynamicMoveSolveRPMThreshold = 3;

    /**
     * The time threshold for convergence in dynamic motion solve.
     */
    public static double kDynamicMoveSolveTimeThreshold = 0.05;

    /**
     * The maximum noise in the roll (X axis rotation) for projectile visualization. Do not
     * increase too much due to it causing choppy rotation. Uses radians as the unit of angle.
     */
    public static double kMaxRollNoiseRad = Math.toRadians(2);

    /**
     * The list of projectile targets to use for trajectory visualization.
     */
    public static List<ProjectileTarget> kProjectileTargets = List.of();

    /**
     * The default tolerance for the projectile target in meters.
     */
    public static double kDefaultProjectileTolerance = 0.14;

    /**
     * The maximum time-of-flight to simulate for a given object.
     */
    public static double kTrajectoryMaxTime = 10;

    /**
     * A constant velocity to apply when the velocity is too low to avoid the projectile getting
     * stuck.
     */
    public static double kVelocityEpsilon = 0.1;

    /**
     * The threshold to apply the velocity compensation.
     */
    public static double kVelocityEpsilonThreshold = 0.1;

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
     * Put zero for the kinematics parameters to calculate static state.
     *
     * <p>
     * Note: The method may be slightly performance intensive if called periodically.
     * <p>
     * <b>Note: The turret yaw assumes left is positive, as per WPILib coordinate systems. Negate
     * the angle if your turret uses right as positive.</b>
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
        // if (!state.isPossible()) { return new ShooterState(0, 0, 0, false, 0); }

        double rpm = state.getWheelRPM();
        double t = (m_useSpinup ? Math.max(
                Math.abs(m_descriptor.getWindupTimes().get(rpm) - m_descriptor.getWindupTimes().get(currentRPM)), 0)
                * m_descriptor.getNominalVoltage() / getBatteryVoltage() : 0) + m_descriptor.getHopperDelay();

        // Converge via iteration
        int iteration = 0;
        while (iteration++ < kDynamicMoveSolveIterations) {
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
            t = (m_useSpinup ? Math.max(
                    Math.abs(m_descriptor.getWindupTimes().get(rpm) - m_descriptor.getWindupTimes().get(currentRPM)), 0)
                    * m_descriptor.getNominalVoltage() / getBatteryVoltage() : 0) + m_descriptor.getHopperDelay();

            // Convergence check
            if (Math.abs(rpm - lastRPM) < kDynamicMoveSolveRPMThreshold
                    && Math.abs(t - lastT) < kDynamicMoveSolveTimeThreshold) {
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
    public static List<ShooterProjectile> updateVisualizer(String loggerKey, List<ShooterProjectile> list) {
        List<Pose3d> poses = new ArrayList<>();
        List<ShooterProjectile> newProjectiles = new ArrayList<>();
        for (ShooterProjectile projectile : list) {
            // Timer not initialized, first run
            if (projectile.lastUpdate == -1) {
                projectile.lastUpdate = Timer.getFPGATimestamp();
                projectile.shotAt = Timer.getFPGATimestamp();
            }
            // No physics updates for frozen projectiles
            if (projectile.isFrozen) { newProjectiles.add(projectile); poses.add(projectile.pose); continue; }

            double now = Timer.getFPGATimestamp();
            double dt = now - projectile.lastUpdate;
            // Compute new state
            projectile.velocity = projectile.velocity.add(projectile.acceleration.scale(dt));
            // Linear approximation: dx = v * t + 1/2 at^2
            Vector3 dx = projectile.velocity.scale(dt).add(projectile.acceleration.scale(0.5 * dt * dt));
            Pose3d lastPose = projectile.pose; // Save pose
            projectile.pose = new Pose3d(projectile.pose.getX() + dx.x, projectile.pose.getY() + dx.y,
                    projectile.pose.getZ() + dx.z, rotationFromVelocity(projectile.velocity));

            // Check target hits
            boolean destroy = false;
            for (ProjectileTarget target : kProjectileTargets) {
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
            if (projectile.pose.getZ() >= 0 && Timer.getFPGATimestamp() - projectile.shotAt <= kTrajectoryMaxTime) {
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
        // Calculate offsets
        var rel = targetPose.relativeTo(robotPose).getTranslation();
        double dx = rel.getX();
        double dy = rel.getY();
        double dz = rel.getZ() - m_descriptor.getPivotHeight();

        // Static geometry
        double horizontalDist = Math.hypot(dx, dy);
        double hoodAngleDeg = m_descriptor.getAngleInterpolator().interpolate(horizontalDist, dz);

        // Field-relative exit speeds
        double v_fr = computeExitVelocityFromRPM(m_descriptor.getRPMInterpolator().interpolate(horizontalDist, dz),
                proj);

        @SuppressWarnings("unused")
        double t, ux, uy; // Time, and aim vectors
        try {
            double[] sol = intercept2D(dx, dy, chassisVx, chassisVy, v_fr);
            t = sol[0];
            ux = sol[1];
            uy = sol[2];
        } catch (IllegalArgumentException e) {
            // no intercept possible
            return new ShooterState(hoodAngleDeg, 0, 0, false, 0);
        }

        // Projecting the velocities along the shot direction:
        // Given vx, vy; the projected velocity is their scalar (dot) product:
        // v_a = <vx, vy> . (<tx, ty> - <rx, ry>) <--- this vector points from robot to target
        // which is equivalent to v_a = vx * cosA + vy * sinA, where A = arctan(dy / dx).
        // This can be represented in fractional notation as:
        // v_a = (vx * dx + vy * dy) / sqrt(dx^2 + dy^2)
        // But in our case, the vectors are computed and given as ux and uy due to yaw, instead of:
        // v_a = (chassisVx * dx + chassisVy * dy) / Math.sqrt(dx * dx + dy * dy);
        // we can and should do the following, to account for yaw:
        double v_along = chassisVx * ux + chassisVy * uy;
        // Robot-relative speeds:
        // v_rr = v_fr - v_a, since the velocity along the shot axis will compensate for the lowered
        // RPM and exit velocity
        double v_rr = v_fr - v_along;
        // To avoid projectile getting stuck just in case, apply v_eps
        if (v_rr < 0 && v_rr >= -kVelocityEpsilonThreshold) v_rr -= kVelocityEpsilon;
        else if (v_rr >= 0 && v_rr <= kVelocityEpsilonThreshold) v_rr += kVelocityEpsilon;

        double rpmFinal = computeRPMFromExitVelocity(v_rr, proj);

        // Calculate yaw
        // Instead of: double yawRad = Math.atan2(dy, dx); we do:
        double yawRad = Math.atan2(uy, ux);
        // Use rads first for relative angle
        double yawRel = yawRad - robotPose.toPose2d().getRotation().getRadians();
        // Finalize with degrees
        yawRel = Math.toDegrees(MathUtil.angleModulus(yawRel));

        boolean isPossible = true;
        // Check for RPM limits and hardware limits
        if (rpmFinal > m_descriptor.getMaxRPM()) {
            isPossible = false;
        } else if (hoodAngleDeg > m_descriptor.getMaxAngle() || hoodAngleDeg < m_descriptor.getMinAngle()) {
            // Check angle limits
            isPossible = false;
        }

        return new ShooterState(hoodAngleDeg, yawRel, rpmFinal, isPossible, v_fr);
    }

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

    private double computeExitVelocityFromRPM(double rpm, ShooterProjectileType proj) {
        double moi = getTotalInertia();
        double w = rpm * kRPMRadCF;
        return proj.getECoeff() * w * m_descriptor.getRollerRadius() * moi / (moi + proj.getEffectiveInertia());
    }

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
    private static Rotation3d rotationFromVelocity(Vector3 v) {
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

    private static double randomRoll() {
        return Math.random() * (2 * kMaxRollNoiseRad) - kMaxRollNoiseRad;
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
        // Discriminant
        double disc = b * b - 4 * a * c;
        if (disc < 0) throw new IllegalArgumentException("no intercept possible");
        // Solve quadratic
        double sqrtD = Math.sqrt(disc);
        double t1 = (-b + sqrtD) / (2 * a);
        double t2 = (-b - sqrtD) / (2 * a);
        // Pick smallest positive
        double t = Double.POSITIVE_INFINITY; // Default invalid
        if (t1 > 0) t = Math.min(t, t1);
        if (t2 > 0) t = Math.min(t, t2);
        if (!Double.isFinite(t)) throw new IllegalArgumentException("no positive intercept time");
        // Direction vector <ux, uy>
        double ux = (dx - vx * t) / (s * t);
        double uy = (dy - vy * t) / (s * t);
        // Normalize (just in case of numerical drift)
        double norm = Math.hypot(ux, uy);
        return new double[] { t, ux / norm, uy / norm };
    }
}
