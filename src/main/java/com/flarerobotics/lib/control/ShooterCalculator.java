package com.flarerobotics.lib.control;

import com.flarerobotics.lib.Utils;
import com.flarerobotics.lib.math.BilinearInterpolator2D;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Computes optimal shooter hood angle and flywheel RPM (unloaded) to hit a
 * specified target using the given calculation strategy.
 */
public class ShooterCalculator {

    private ShooterConfig m_config;

    private CalculationStrategy m_calculationStrategy = CalculationStrategy.FULLY_EMPIRICAL;

    /**
     * Constructs a new ShooterCalculator.
     *
     * @param config The configuration for the shooter.
     */
    public ShooterCalculator(ShooterConfig config) {
        m_config = config;
        m_calculationStrategy = config.getConstructorCalculationStrategy();
    }

    // Public Variables //

    /**
     * The amount of iterations to go through to solve for the shooter state.
     * Increase if more accuracy is desired, but may impact performance. Defaults to
     * 5.
     */
    public static int kSolveIterations = 5;

    /**
     * Whether to warn for convergence issues. If true, will warn if the
     * the angle does not converge fully within the specified number of iterations,
     * determined by {@link #kConvergenceThresholdRadians}. May cause unwanted logs,
     * so is disabled by default. Recommended to enable while debugging.
     */
    public static boolean kWarnForConvergence = false;

    /**
     * The convergence threshold for the angle. If the difference between the
     * current and last angle is less than this, the angle is considered converged.
     * Defaults to 0.017 radians (1 degree).
     */
    public static double kConvergenceThresholdRadians = 0.017;

    /**
     * The air density, in kg/m^3. You most likely don't need to change this.
     */
    public static double kAirDensity = 1.2;

    /**
     * Returned when an error occurs while calculating the shooter state.
     */
    public static final ShooterState kInvalidState = new ShooterState(45, 0, false, Pose3d.kZero, 0, false);

    /**
     * The battery voltage to use for unit tests. Defaults to 12.5V.
     */
    public static final double kTestBatteryVoltage = 12.5;

    // Private Constants //

    private static final double kG = 9.80665; // m/s^2

    // Public Methods //

    /**
     * Computes the shooter state to hit a target, given the inputs and targets.
     *
     * <p>
     * <b>Notes:</b>
     * <p>
     * - The system does not account for a moving chassis. <b>Call periodically</b>
     * to update if the chassis is moving, or if it takes a long enough time to
     * adjust the hood/barrel.
     * <p>
     * - Keep in mind there may be <b>serious inaccuracies</b> due to air drag for
     * <b>partially-empirical</b> data.
     * <p>
     * - The robot pose and target pose must be <b>field-relative.</b>
     *
     * @param robotPose  The pose of the robot.
     * @param targetPose The pose of the target.
     * @return The computed shooter state.
     */
    public ShooterState calculate(Pose3d robotPose, Pose3d targetPose) {
        switch (m_calculationStrategy) {
            case PARTIAL_EMPIRICAL:
                return calculate_partialEmpirical(robotPose, targetPose);
            case FULLY_EMPIRICAL:
                return calculate_fullyEmpirical(robotPose, targetPose);
            default:
                DriverStation.reportWarning(
                        "ShooterCalculator::calculate, invalid calculation strategy: " + m_calculationStrategy, true);
                return kInvalidState;
        }
    }

    /**
     * Returns the simulated trajectory for the game piece. <b>Accounts for
     * drag.</b>
     *
     * @param state           The shooter state.
     * @param simFrequency    The capture frequency in Hertz. (e.g. 50Hz for 0.02s
     *                        sim period)
     * @param maxTime         The maximum time for the simulation in seconds.
     * @param mass            The mass of the game piece in kg.
     * @param surfaceArea     The surface area of the game piece in m^2.
     * @param dragCoefficient The drag coefficient of the game piece.
     *                        Typical values are around 0.47 for a sphere, 1.05 for
     *                        a cube, and 0.04-0.09 for streamlined objects.
     * @return The simulated trajectory of the game piece.
     */
    public static List<Pose3d> simulateTrajectory(
            ShooterState state, // the shooter state
            double simFrequency, // simulation frequency (e.g. 50Hz for 0.02s)
            double maxTime, // maximum time (seconds)
            double mass,
            double surfaceArea,
            double dragCoefficient) {
        return simulateTrajectoryPrivate(state, simFrequency, maxTime, true, mass, surfaceArea, dragCoefficient);
    }

    /**
     * Returns the simulated trajectory for the game piece. <b>Ignores drag.</b>
     *
     * @param state        The shooter state.
     * @param simFrequency The capture frequency in Hertz. (e.g. 50Hz for 0.02s sim
     *                     period)
     * @param maxTime      The maximum time for the simulation in seconds.
     * @return The simulated trajectory of the game piece.
     */
    public static List<Pose3d> simulateTrajectory(
            ShooterState state, // the shooter state
            double simFrequency, // simulation frequency (e.g. 50Hz for 0.02s)
            double maxTime // maximum time (seconds)
            ) {
        return simulateTrajectoryPrivate(state, simFrequency, maxTime, false, 0, 0, 0);
    }

    /**
     * Returns the shooter configuration.
     *
     * @return The configuration.
     */
    public ShooterConfig getConfig() {
        return m_config;
    }

    // Private Helper Methods //

    private ShooterState calculate_partialEmpirical(Pose3d robotPose, Pose3d targetPose) {
        // Base geometry to pivot //
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double dBase = Math.hypot(dx, dy);
        double dzBase = targetPose.getZ() - (robotPose.getZ() + m_config.getPivotHeight());

        // Initial rough guess for angle //
        double theta;
        try {
            theta = computeOptimalAngleRough(dBase, dzBase);
        } catch (IllegalArgumentException e) {
            DriverStation.reportWarning(
                    "ShooterCalculator::calculate, the target is unreachable, check the target height and distance",
                    true);
            return kInvalidState;
        }

        if (Double.isNaN(theta)) {
            DriverStation.reportWarning("NaN theta detected", true);
            return kInvalidState;
        }

        double dEff = dBase, dzEff = dzBase;
        double vExit = 0.0;

        boolean converged = false;

        // Iterate to converge dEff/dzEff ↔ vExit ↔ θ //
        for (int i = 0; i < kSolveIterations; i++) {
            // Previous angle for convergence check
            double lastTheta = theta;

            // a) adjust for barrel tilt
            dEff = dBase - m_config.getBarrelLength() * Math.cos(theta);
            dzEff = dzBase + m_config.getBarrelLength() * Math.sin(theta);

            // b) lookup **unloaded** RPM from your empirical map
            double rpmUnloaded = m_config.getFreeSpeedInterpolator().interpolate(dEff, dzEff);

            // c) compute the **loaded** RPM after the game object hits
            double rpmLoaded = rpmUnloaded
                    * (1.0
                            - m_config.getDropFractionMap().get(rpmUnloaded)
                                    * (m_config.getNominalVoltage() / getBatteryVoltage())); // Multiply by V_n/V_c to
            // scale inversely with V_c

            // d) convert loaded RPM → exit speed
            vExit = rpmToSpeed(rpmLoaded, m_config.getWheelRadius());

            // e) solve gravity-based projectile formula for θ
            try {
                theta = computeLaunchAngle(dEff, dzEff, vExit);
            } catch (IllegalArgumentException e) {
                DriverStation.reportWarning(
                        "ShooterCalculator::calculate, the target is unreachable, check the target height and distance: "
                                + e.getMessage(),
                        true);
                return kInvalidState; // Or break the loop
            }

            // f) check for convergence
            if (i > 0 && !Double.isNaN(lastTheta) && Math.abs(theta - lastTheta) <= kConvergenceThresholdRadians) {
                converged = true;
                break;
            }
        }

        if (kWarnForConvergence && !converged)
            DriverStation.reportWarning(
                    "ShooterCalculator::calculate, the final angle may not have converged fully, consider increasing the iteration count in account to performance.",
                    true);

        // Final commanded RPM is the unloaded RPM from the map //
        double finalRPM = m_config.getFreeSpeedInterpolator().interpolate(dEff, dzEff);
        double thetaDegs = Math.toDegrees(theta);

        // Final validity checks //
        if (Math.abs(finalRPM) > m_config.getMaxFreeRPM()) {
            DriverStation.reportWarning("ShooterCalculator::calculate, the target RPM exceeds the max RPM", true);
            return kInvalidState;
        }

        if (thetaDegs > m_config.getMaxAngle() || thetaDegs < m_config.getMinAngle()) {
            DriverStation.reportWarning(
                    "ShooterCalculator::calculate, the hood/barrel angle out of bounds: " + thetaDegs, true);
            return kInvalidState;
        }

        // Calculate the end point pose of the hood/barrel //
        Transform3d pivotTransform =
                new Transform3d(new Translation3d(0, 0, m_config.getPivotHeight()), new Rotation3d());

        double horizontalAngleRads =
                Math.toRadians(m_config.getPivotHorizontalRotationSupplier().get());
        Transform3d hoodTransform = new Transform3d(
                new Translation3d(m_config.getBarrelLength(), 0, 0), new Rotation3d(0, theta, horizontalAngleRads));

        return new ShooterState(
                thetaDegs,
                finalRPM,
                true,
                robotPose.transformBy(pivotTransform).transformBy(hoodTransform),
                vExit,
                converged);
    }

    private ShooterState calculate_fullyEmpirical(Pose3d robotPose, Pose3d targetPose) {
        if (m_config.getAngleInterpolator() == null) {
            DriverStation.reportWarning(
                    "ShooterCalculator::calculate, the angle interpolator is not set, cannot calculate the angle",
                    true);
            return kInvalidState;
        }
        // Compute raw horizontal distance & height difference at pivot //
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double dBase = Math.hypot(dx, dy);
        double dzBase = targetPose.getZ() - (robotPose.getZ() + m_config.getPivotHeight());

        // Converge on the true muzzle-to-target geometry //
        double thetaRad = 0.0;
        double dEff = dBase, dzEff = dzBase;
        boolean converged = false;
        for (int i = 0; i < kSolveIterations; i++) {
            double lastTheta = thetaRad;
            // a) lookup hood-angle from your 2D angle map
            double angleDeg = m_config.getAngleInterpolator().interpolate(dEff, dzEff);
            thetaRad = Math.toRadians(angleDeg);

            // b) shift effective horizontal & vertical offsets by barrel tilt
            dEff = dBase - m_config.getBarrelLength() * Math.cos(thetaRad);
            dzEff = dzBase + m_config.getBarrelLength() * Math.sin(thetaRad);

            // c) check for convergence
            if (i > 0 && !Double.isNaN(lastTheta) && Math.abs(thetaRad - lastTheta) <= kConvergenceThresholdRadians) {
                converged = true;
                break;
            }
        }

        if (kWarnForConvergence && !converged)
            DriverStation.reportWarning(
                    "ShooterCalculator::calculate, the final angle may not have converged fully, consider increasing the iteration count in account to performance.",
                    true);

        // Final lookups: hood angle & unloaded RPM //
        double finalAngleDeg = m_config.getAngleInterpolator().interpolate(dEff, dzEff);
        double finalRPM = m_config.getFreeSpeedInterpolator().interpolate(dEff, dzEff);

        // c) compute the **loaded** RPM after the game object hits
        double rpmLoaded = finalRPM
                * (1.0
                        - m_config.getDropFractionMap().get(finalRPM)
                                * (m_config.getNominalVoltage() / getBatteryVoltage())); // Multiply by V_n/V_c to scale
        // inversely with V_c

        // d) convert loaded RPM → exit speed
        double vExit = rpmToSpeed(rpmLoaded, m_config.getWheelRadius());

        // Final validity checks //
        if (Math.abs(finalRPM) > m_config.getMaxFreeRPM()) {
            DriverStation.reportWarning("ShooterCalculator::calculate, the target RPM exceeds the max RPM", true);
            return kInvalidState;
        }

        if (thetaRad > Math.toRadians(m_config.getMaxAngle()) || thetaRad < Math.toRadians(m_config.getMinAngle())) {
            DriverStation.reportWarning(
                    "ShooterCalculator::calculate, the hood/barrel angle out of bounds: " + Math.toDegrees(thetaRad),
                    true);
            return kInvalidState;
        }

        // Calculate the end point pose of the hood/barrel //
        Transform3d pivotTransform =
                new Transform3d(new Translation3d(0, 0, m_config.getPivotHeight()), new Rotation3d());

        double horizontalAngleRads =
                Math.toRadians(m_config.getPivotHorizontalRotationSupplier().get());
        Transform3d hoodTransform = new Transform3d(
                new Translation3d(m_config.getBarrelLength(), 0, 0),
                new Rotation3d(0, Math.toRadians(finalAngleDeg), horizontalAngleRads));

        return new ShooterState(
                finalAngleDeg,
                finalRPM,
                true,
                robotPose.transformBy(pivotTransform).transformBy(hoodTransform),
                vExit,
                converged);
    }

    /**
     * Solve for the launch angle θ (radians) that sends
     * a projectile of speed v through horizontal d and vertical dz:
     *
     * <p>
     * dz = d*tanθ - (g * d^2) / (2 v^2 cos^2θ)
     *
     * Returns the correct solution.
     */
    private static double computeLaunchAngle(double d, double dz, double v) {
        if (d <= 0) {
            if (d == 0 && dz == 0) return 0;
            throw new IllegalArgumentException("invalid d or dz value d='" + d + "' dz='" + dz + "'");
        }
        if (v <= 0) throw new IllegalArgumentException("invalid v value '" + v + "'");

        double v2 = v * v;
        double discriminant = v2 * v2 - kG * (kG * d * d + 2 * dz * v2);

        if (discriminant < 0) {
            throw new IllegalArgumentException("no valid angle for v=" + v + ", discriminant < 0");
        }

        // Two solutions: (v² ± √disc) / (g d)
        double root = Math.sqrt(discriminant);
        double tHigh = (v2 + root) / (kG * d);
        double tLow = (v2 - root) / (kG * d);

        // atan of both; choose the correct root
        double solHigh = Math.atan(tHigh);
        double solLow = Math.atan(tLow);
        // return Math.min(sol1, sol2);

        // We want to allow downward shots whenever the target is below us:
        if (dz < 0) {
            // If solLow is negative, this is your downward shot
            if (solLow <= 0) {
                return solLow;
            }
            // Otherwise fall back to the less‑negative one
            return solHigh <= 0 ? solHigh : solLow;
        }

        // Prefer the low-arc if it’s non-negative
        if (solLow >= 0) {
            return solLow;
        }
        // Otherwise, if the high-arc is non-negative, use that
        if (solHigh >= 0) {
            return solHigh;
        }

        // Invalid angles, throw exception
        throw new IllegalArgumentException(String.format("invalid angles (lo=%.3f, hi=%.3f)", solLow, solHigh));
    }

    /** Rough initial guess for the launch angle */
    private static double computeOptimalAngleRough(double d, double dz) {
        double root = Math.sqrt(d * d + dz * dz);

        // Using epsilonEquals to avoid floating-point inaccuracy errors
        if (Utils.epsilonEquals(dz + root, 0)) {
            throw new IllegalArgumentException("invalid geometry for angle");
        }

        return Math.atan(d / (dz + root));
    }

    /** Helper to convert RPM → linear speed at wheel rim */
    private double rpmToSpeed(double rpm, double r) {
        double w = rpm * 2 * Math.PI / 60.0; // rad/s
        return w * r * m_config.getEfficiency(); // m/s
    }

    private static List<Pose3d> simulateTrajectoryPrivate(
            ShooterState state, // the shooter state
            double simFrequency, // simulation frequency (e.g. 50Hz for 0.02s)
            double maxTime, // maximum time (seconds)
            boolean useDrag,
            double mass,
            double surfaceArea,
            double dragCoefficient) {
        simFrequency = 1 / simFrequency; // Convert to period (s) from Hz(s^-1)
        List<Pose3d> traj = new ArrayList<>();

        // Get data from the shooter state
        Pose3d exitPose = state.getShooterPosition();
        double vExit = state.getExitVelocity();
        double barrelYawDegs = state.getHoodAngleDeg();

        // Initial state
        double x = exitPose.getX();
        double y = exitPose.getY();
        double z = exitPose.getZ();
        // Decompose vExit into components
        double pitch = exitPose.getRotation().getY(); // pitch
        double yaw = Math.toRadians(barrelYawDegs); // yaw
        double vx = vExit * Math.cos(pitch) * Math.cos(yaw);
        double vy = vExit * Math.cos(pitch) * Math.sin(yaw);
        double vz = vExit * Math.sin(pitch);

        double t = 0.0;
        while (z >= 0 && t <= maxTime) {
            // Record current pose
            double horizSpeed = Math.sqrt(vx * vx + vy * vy);
            Rotation3d rot = new Rotation3d(0, Math.atan2(vz, horizSpeed), Math.atan2(vy, vx));
            traj.add(new Pose3d(new Translation3d(x, y, z), rot));

            // Compute acceleration
            double ax = 0, ay = 0, az = -kG;
            if (useDrag) {
                double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
                double fDrag = 0.5 * kAirDensity * dragCoefficient * surfaceArea * speed * speed;
                if (speed > 1e-9) {
                    ax = -fDrag * (vx / speed) / mass;
                    ay = -fDrag * (vy / speed) / mass;
                    az += -fDrag * (vz / speed) / mass;
                }
            }
            // Euler integration
            vx += ax * simFrequency;
            vy += ay * simFrequency;
            vz += az * simFrequency;
            x += vx * simFrequency;
            y += vy * simFrequency;
            z += vz * simFrequency;
            t += simFrequency;
        }

        return traj;
    }

    private static double getBatteryVoltage() {
        // Was getting access exceptions for RoboRioSim when unit testing, this was the
        // fix.
        if (RobotState.isTest()) {
            return kTestBatteryVoltage;
        }
        if (Robot.isSimulation()) {
            return RoboRioSim.getVInVoltage();
        } else {
            return RobotController.getBatteryVoltage();
        }
    }

    // Classes && Enums

    /** Configuration inputs for the shooter mechanism. */
    public static class ShooterConfig {
        private final double barrelLength; // meters
        private final double pivotHeight; // meters (height of pivot above ground)
        private final double wheelRadius; // meters
        private final InterpolatingDoubleTreeMap dropFractionMap; // empirical load drop fraction
        private final double maxFreeRPM;
        private final double maxAngle;
        private final double minAngle;
        private final double efficiency;
        private final double nominalVoltage;
        private final Supplier<Double> pivotHorizontalRotation;
        private final BilinearInterpolator2D angleInterpolator;
        private final BilinearInterpolator2D RPMInterpolator;

        private final CalculationStrategy calculationStrategy;

        /**
         * Constructs a new fully-empirical (see {@link CalculationStrategy})
         * ShooterConfig.
         *
         * <p>
         * <b>Example Arrays: </b>
         *
         * <pre>
         * <code>
         * // Sorted Arrays
         * double[] horizontalDistances = { 2.0, 3.0, 4.0, 5.0 }; // meters
         * double[] heightDiffs = { -0.5, 0.0, +0.5, +1.0 }; // meters, relative to the end point of the hood/barrel
         * double[][] angleGrid = {
         *         // -----------------> HeightDiff (-0.5 -> +1.0)
         *         { 45, 40, 35, 30 }, // at d=2.0
         *         { 42, 38, 33, 28 }, // at d=3.0
         *         { 39, 35, 31, 27 }, // at d=4.0
         *         { 37, 33, 29, 25 }, // at d=5.0
         * };
         * double[][] rpmGrid = {
         *         // -------------------------> HeightDiff (-0.5 -> +1.0)
         *         { 3000, 3100, 3200, 3300 }, // at d=2.0
         *         { 2900, 3000, 3100, 3200 }, // at d=3.0
         *         { 2800, 2900, 3000, 3100 }, // at d=4.0
         *         { 2700, 2800, 2900, 3000 }, // at d=5.0
         * };
         * </code>
         * </pre>
         *
         * <p>
         * <i><b>Notes:</b></i>
         * <p>
         * <i>
         * <b>
         * - The emprical data must be measured from the shooter's pivot point.
         * </b>
         * </i>
         * <p>
         * <i>
         * <b>
         * - The grids must be sorted in ascending order.
         * </b>
         * </i>
         * <p>
         * <i>
         * <b>
         * - The data should be large enough to avoid extrapolation due to out-of-bounds
         * data, which may lead to significant inacuracies.
         * </b>
         * </i>
         * <p>
         * <i>
         * <b>
         * - The RPM speeds use the flywheel RPM.
         * </b>
         * </i>
         *
         * @param barrelLength                    The length of the barrel/hood in
         *                                        meters.
         * @param pivotHeight                     The baseline height (pivot height) in
         *                                        meters.
         * @param wheelRadius                     The radius of the flywheel in meters.
         * @param dropFractionMap                 The fractional drop from unloaded to
         *                                        loaded RPM: (RPM_unloaded -
         *                                        RPM_loaded) / RPM_unloaded. Not used
         *                                        when the calculation mode is fully
         *                                        empirical. Map of unloaded RPM to the
         *                                        drop fraction.
         * @param maxFreeRPM                      The maximum free RPM of the flywheel.
         * @param maxAngle                        The maximum angle of the hood in
         *                                        degrees.
         * @param minAngle                        The minimum angle of the hood in
         *                                        degrees.
         * @param efficiency                      The efficiency of the system on a
         *                                        scale of 0->1. Defined as: v_real /
         *                                        v_theory. (RPM -> Velocity conversion
         *                                        efficiency)
         * @param nominalVoltage                  The nominal voltage of the battery.
         * @param pivotHorizontalRotationSupplier The supplier for the horizontal
         *                                        rotation of the shooter pivot in
         *                                        degrees, in account to robot rotation.
         *                                        Use the robot's heading if the shooter
         *                                        doesn't pivot horizontally. Otherwise,
         *                                        add the horizontal angle of the
         *                                        shooter to the robot's heading.
         * @param distances                       The horizontal distances in meters for
         *                                        the empirical map, with width M.
         * @param heightDiffs                     The height differences in meters for
         *                                        the empirical map, with width N.
         * @param angles                          The angles in degrees for the
         *                                        empirical map, in WxH = NxM size. Must
         *                                        be sorted.
         * @param freeSpeeds                      The free speeds in RPM for the
         *                                        empirical map, in WxH = NxM size. Must
         *                                        be sorted.
         */
        public ShooterConfig(
                double barrelLength,
                double pivotHeight,
                double wheelRadius,
                InterpolatingDoubleTreeMap dropFractionMap,
                double maxFreeRPM,
                double maxAngle,
                double minAngle,
                double efficiency,
                double nominalVoltage,
                Supplier<Double> pivotHorizontalRotationSupplier,
                double[] distances,
                double[] heightDiffs,
                double[][] angles,
                double[][] freeSpeeds) {
            if (distances == null
                    || heightDiffs == null
                    || angles == null
                    || freeSpeeds == null
                    || dropFractionMap == null) {
                throw new IllegalArgumentException("null arrays are not allowed");
            }
            if (barrelLength < 0 || pivotHeight < 0 || wheelRadius <= 0 || maxFreeRPM <= 0 || maxAngle <= 0) {
                throw new IllegalArgumentException("invalid shooter config parameters");
            }
            if (minAngle > maxAngle) {
                throw new IllegalArgumentException("minAngle must be less than or equal to maxAngle");
            }
            if (angles.length != distances.length || angles[0].length != heightDiffs.length) {
                throw new IllegalArgumentException("the angles grid has invalid dimensions");
            }
            if (freeSpeeds.length != distances.length || freeSpeeds[0].length != heightDiffs.length) {
                throw new IllegalArgumentException("the free-speed grid has invalid dimensions");
            }
            this.barrelLength = barrelLength;
            this.pivotHeight = pivotHeight;
            this.wheelRadius = wheelRadius;
            this.dropFractionMap = dropFractionMap;
            this.maxFreeRPM = maxFreeRPM;
            this.maxAngle = maxAngle;
            this.minAngle = minAngle;
            this.efficiency = efficiency;
            this.nominalVoltage = nominalVoltage;
            this.pivotHorizontalRotation = pivotHorizontalRotationSupplier;
            this.angleInterpolator = new BilinearInterpolator2D(distances, heightDiffs, angles);
            this.RPMInterpolator = new BilinearInterpolator2D(distances, heightDiffs, freeSpeeds);
            this.calculationStrategy = CalculationStrategy.FULLY_EMPIRICAL;
        }

        /**
         * Constructs a new partially-empirical (see {@link CalculationStrategy})
         * ShooterConfig. Preferrably use {@link CalculationStrategy#FULLY_EMPIRICAL}
         * instead.
         *
         * <p>
         * <b>Example Arrays: </b>
         *
         * <pre>
         * <code>
         * // Sorted Arrays
         * double[] horizontalDistances = { 2.0, 3.0, 4.0, 5.0 }; // meters
         * double[] heightDiffs = { -0.5, 0.0, +0.5, +1.0 }; // meters, relative to the end point of the hood/barrel
         * double[][] rpmGrid = {
         *         // -------------------------> HeightDiff (-0.5 -> +1.0)
         *         { 3000, 3100, 3200, 3300 }, // at d=2.0
         *         { 2900, 3000, 3100, 3200 }, // at d=3.0
         *         { 2800, 2900, 3000, 3100 }, // at d=4.0
         *         { 2700, 2800, 2900, 3000 }, // at d=5.0
         * };
         * </code>
         * </pre>
         *
         * <p>
         * <i><b>Notes:</b></i>
         * <p>
         * <i>
         * <b>
         * - The emprical data must be measured from the shooter's pivot point.
         * </b>
         * </i>
         * <p>
         * <i>
         * <b>
         * - The angle grid must be sorted in ascending order.
         * </b>
         * </i>
         * <p>
         * <i>
         * <b>
         * - The data should be large enough to avoid extrapolation due to out-of-bounds
         * data.
         * </b>
         * </i>
         * <p>
         * <i>
         * <b>
         * - The RPM speeds use the flywheel RPM.
         * </b>
         * </i>
         *
         * @param barrelLength                    The length of the barrel/hood in
         *                                        meters.
         * @param pivotHeight                     The baseline height (pivot height) in
         *                                        meters.
         * @param wheelRadius                     The radius of the flywheel in meters.
         * @param dropFractionMap                 The fractional drop from unloaded to
         *                                        loaded RPM: (RPM_unloaded -
         *                                        RPM_loaded) / RPM_unloaded. Not used
         *                                        when the calculation mode is fully
         *                                        empirical. Map of unloaded RPM to the
         *                                        drop fraction.
         * @param maxFreeRPM                      The maximum free RPM of the flywheel.
         * @param maxAngle                        The maximum angle of the hood in
         *                                        degrees.
         * @param minAngle                        The minimum angle of the hood in
         *                                        degrees.
         * @param efficiency                      The efficiency of the system on a
         *                                        scale of 0->1. Defined as: v_real /
         *                                        v_theory. (RPM -> Velocity conversion
         *                                        efficiency)
         * @param nominalVoltage                  The nominal voltage of the battery.
         * @param pivotHorizontalRotationSupplier The supplier for the horizontal
         *                                        rotation of the shooter pivot in
         *                                        degrees, in account to robot rotation.
         *                                        Use the robot's heading if the shooter
         *                                        doesn't pivot horizontally. Otherwise,
         *                                        add the horizontal angle of the
         *                                        shooter to the robot's heading.
         * @param distances                       The horizontal distances in meters for
         *                                        the empirical map, with width M.
         * @param heightDiffs                     The height differences in meters for
         *                                        the empirical map, with width N.
         * @param freeSpeeds                      The free speeds in RPM for the
         *                                        empirical map, in WxH = NxM size. Must
         *                                        be sorted.
         */
        public ShooterConfig(
                double barrelLength,
                double pivotHeight,
                double wheelRadius,
                InterpolatingDoubleTreeMap dropFractionMap,
                double maxFreeRPM,
                double maxAngle,
                double minAngle,
                double efficiency,
                double nominalVoltage,
                Supplier<Double> pivotHorizontalRotationSupplier,
                double[] distances,
                double[] heightDiffs,
                double[][] freeSpeeds) {
            if (distances == null || heightDiffs == null || freeSpeeds == null || dropFractionMap == null) {
                throw new IllegalArgumentException("null arrays are not allowed");
            }
            if (barrelLength < 0 || pivotHeight < 0 || wheelRadius <= 0 || maxFreeRPM <= 0 || maxAngle <= 0) {
                throw new IllegalArgumentException("invalid shooter config parameters");
            }
            if (minAngle > maxAngle) {
                throw new IllegalArgumentException("minAngle must be less than or equal to maxAngle");
            }
            if (freeSpeeds.length != distances.length || freeSpeeds[0].length != heightDiffs.length) {
                throw new IllegalArgumentException("the free-speed grid has invalid dimensions");
            }
            this.barrelLength = barrelLength;
            this.pivotHeight = pivotHeight;
            this.wheelRadius = wheelRadius;
            this.dropFractionMap = dropFractionMap;
            this.maxFreeRPM = maxFreeRPM;
            this.maxAngle = maxAngle;
            this.minAngle = minAngle;
            this.efficiency = efficiency;
            this.nominalVoltage = nominalVoltage;
            this.pivotHorizontalRotation = pivotHorizontalRotationSupplier;
            this.angleInterpolator = null;
            this.RPMInterpolator = new BilinearInterpolator2D(distances, heightDiffs, freeSpeeds);
            this.calculationStrategy = CalculationStrategy.PARTIAL_EMPIRICAL;
        }

        public double getBarrelLength() {
            return barrelLength;
        }

        public double getPivotHeight() {
            return pivotHeight;
        }

        public double getWheelRadius() {
            return wheelRadius;
        }

        public InterpolatingDoubleTreeMap getDropFractionMap() {
            return dropFractionMap;
        }

        public double getMaxFreeRPM() {
            return maxFreeRPM;
        }

        public double getMaxAngle() {
            return maxAngle;
        }

        public double getMinAngle() {
            return minAngle;
        }

        public double getEfficiency() {
            return efficiency;
        }

        public double getNominalVoltage() {
            return nominalVoltage;
        }

        public Supplier<Double> getPivotHorizontalRotationSupplier() {
            return pivotHorizontalRotation;
        }

        public BilinearInterpolator2D getAngleInterpolator() {
            return angleInterpolator;
        }

        public BilinearInterpolator2D getFreeSpeedInterpolator() {
            return RPMInterpolator;
        }

        public CalculationStrategy getConstructorCalculationStrategy() {
            return calculationStrategy;
        }
    }

    /** Packaging class for computed shooter state. */
    public static class ShooterState {
        private final double m_hoodAngleDeg;
        private final double m_wheelRPM;
        private final boolean m_isPossible;
        private final boolean m_converged;
        private final double m_exitVelocity;

        private final Pose3d m_shooterPosition;

        public ShooterState(
                double hoodAngleDeg,
                double wheelRPM,
                boolean isPossible,
                Pose3d shooterPosition,
                double exitVelocity,
                boolean converged) {
            m_hoodAngleDeg = hoodAngleDeg;
            m_wheelRPM = wheelRPM;
            m_isPossible = isPossible;
            m_converged = converged;
            m_shooterPosition = shooterPosition;
            m_exitVelocity = exitVelocity;
        }

        /**
         * Returns the hood angle in degrees.
         *
         * @return The angle.
         */
        public double getHoodAngleDeg() {
            return m_hoodAngleDeg;
        }

        /**
         * Returns the flywheel RPM.
         *
         * @return The RPM.
         */
        public double getWheelRPM() {
            return m_wheelRPM;
        }

        /**
         * Returns whether the target is reachable, assuming a pivoting shooter.
         *
         * @return true if reachable, false otherwise.
         */
        public boolean isPossible() {
            return m_isPossible;
        }

        /**
         * Returns whether the angle converged.
         *
         * @return true if converged, false otherwise.
         */
        public boolean isConverged() {
            return m_converged;
        }

        /**
         * Returns the shooter position.
         *
         * @return The position.
         */
        public Pose3d getShooterPosition() {
            return m_shooterPosition;
        }

        /**
         * Returns the exit velocity of the game piece.
         *
         * @return The exit velocity.
         */
        public double getExitVelocity() {
            return m_exitVelocity;
        }

        @Override
        public String toString() {
            return String.format("Hood: %.2f° | RPM: %.0f", m_hoodAngleDeg, m_wheelRPM);
        }
    }

    /** The calculation strategy used in {@link ShooterCalculator}. */
    public enum CalculationStrategy {
        /**
         * Uses the free speed RPM provided to calculate the angle. May lead to
         * significant inaccuracies over long distances due to air drag, energy
         * transfer losses and friction. Consider using {@link #FULLY_EMPIRICAL} for
         * more accurate results over long distances.
         */
        PARTIAL_EMPIRICAL,
        /**
         * Uses interpolation to calculate the angle through the given RPM and angle
         * map.
         */
        FULLY_EMPIRICAL,
    }
}
