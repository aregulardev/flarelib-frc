package com.flarerobotics.lib.control;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

/** A shooter angle and velocity calculator. */
public class ShooterCalculator {
    private final double m_shooterHeight;
    private final double m_wheelCircumference;
    private final double m_gearRatio;
    private final double m_maxMotorRpm;
    private final InterpolatingDoubleTreeMap m_rpmLookupTable;

    private double m_desiredTurretYaw;
    private double m_desiredTurretPitch;
    private double m_desiredRPM;
    private boolean m_isFeasible;

    /**
     * Constructs a new ShooterCalculator.
     *
     * @param shooterHeight  The height of the shooter.
     * @param wheelDiameter  The diameter of the wheel.
     * @param gearRatio      The reduction.
     * @param maxMotorRpm    The maximum RPM of the motor in the system.
     * @param rpmLookupTable The interpolating lookup table. Add more data to
     *                       increase precision. (Horizontal Distance(m) - RPM pair
     *                       map)
     */
    public ShooterCalculator(
            Distance shooterHeight,
            Distance wheelDiameter,
            double gearRatio,
            double maxMotorRpm,
            InterpolatingDoubleTreeMap rpmLookupTable) {
        m_shooterHeight = shooterHeight.in(Meters);
        m_wheelCircumference = Math.PI * wheelDiameter.in(Meters);
        m_gearRatio = gearRatio;
        m_maxMotorRpm = maxMotorRpm;
        m_rpmLookupTable = rpmLookupTable;
    }

    /**
     * Updates the shooter parameters.
     *
     * @param targetPosition The position of the shooter's target.
     * @param robotPose      The robot's current pose on the field.
     */
    public void calculateShotParameters(Translation3d targetPosition, Pose2d robotPose) {
        // Calculate relative position
        double dx = targetPosition.getX() - robotPose.getX();
        double dy = targetPosition.getY() - robotPose.getY();
        double horizontalDistance = Math.hypot(dx, dy);
        double verticalDistance = targetPosition.getZ() - m_shooterHeight;

        // Calculate turret yaw
        double fieldRelativeYaw = Math.atan2(dy, dx);
        double robotHeading = robotPose.getRotation().getRadians();
        m_desiredTurretYaw = Math.IEEEremainder(fieldRelativeYaw - robotHeading, 2 * Math.PI);

        // Get base RPM from lookup table
        m_desiredRPM = m_rpmLookupTable.get(horizontalDistance);

        // Apply corrections
        m_isFeasible = m_desiredRPM <= m_maxMotorRpm;

        // Calculate elevation angle
        m_desiredTurretPitch = calculateOptimalAngle(horizontalDistance, verticalDistance);
    }

    /**
     * Calcultes the optimal angle of the shooter based on the RPM.
     *
     * @param horizontalDist The horizontal distance, in meters.
     * @param verticalDist   The vertical distance, in meters.
     * @return The optimal angle, in degrees.
     */
    private double calculateOptimalAngle(double horizontalDist, double verticalDist) {
        // Quadratic solution for angle
        double g = 9.81;
        double v = (m_desiredRPM * m_wheelCircumference) / (60 * m_gearRatio);

        double discriminant =
                Math.pow(v, 4) - g * (g * Math.pow(horizontalDist, 2) + 2 * verticalDist * Math.pow(v, 2));

        // The quadratic has no real solutions if the discriminant < 0.
        if (discriminant < 0) {
            DriverStation.reportWarning(
                    "No angle solution found for trajectory in ShooterCalculator::calculateOptimalAngle, delta < 0",
                    true);
            return 45; // Fallback angle
        }

        double angleRad = Math.atan((Math.pow(v, 2) - Math.sqrt(discriminant)) / (g * horizontalDist));

        return Math.toDegrees(angleRad);
    }

    /**
     * Adds a RPM data point to the interpolating RPM map. Called during
     * testing/characterization.
     *
     * @param distance      The horizontal distance from the target.
     * @param successfulRPM The RPM of the motor.
     */
    public void addEmpiricalDataPoint(Distance distance, double successfulRPM) {
        m_rpmLookupTable.put(distance.in(Meters), successfulRPM);
    }

    /**
     * Returns the desired turret yaw.
     *
     * @return The yaw in degrees.
     */
    public double getDesiredTurretYaw() {
        return m_desiredTurretYaw;
    }

    /**
     * Returns the desired turret pitch.
     *
     * @return The pitch in degrees.
     */
    public double getDesiredTurretPitch() {
        return m_desiredTurretPitch;
    }

    /**
     * Returns the desired RPM of the motor.
     *
     * @return The desired RPM.
     */
    public double getDesiredRPM() {
        return m_desiredRPM;
    }

    /**
     * Returns wether or not the shot possible.
     *
     * @return true if possible.
     */
    public boolean isShotFeasible() {
        return m_isFeasible;
    }
}
