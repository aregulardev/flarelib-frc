package com.flarerobotics.lib.control.positions;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

/**
 * An interface allowing users to have a custom enum of subsystem positions.
 *
 * <p>
 * Useful for storing arm positions along with the elevator positions.
 */
public interface ISubsystemPosition {
    /**
     * For linear mechanism positions.
     *
     * @return The system height.
     */
    default Distance getHeight() {
        return Meters.of(0);
    }

    /**
     * For position-controlled subsystem positions.
     *
     * @param key
     *            The getter key.
     * @return The system height.
     */
    default Distance getHeight(String key) {
        return Meters.of(0);
    }

    /**
     * For position-controlled subsystem positions.
     *
     * @return The system's angle in degrees.
     */
    default double getAngle() {
        return 0;
    }

    /**
     * For position-controlled subsystem positions.
     *
     * @param key
     *            The getter key.
     * @return The system's angle in degrees.
     */
    default double getAngle(String key) {
        return 0;
    }

    /**
     * For angular velocity-controlled subsystem positions.
     *
     * @return The system's RPM.
     */
    default double getRPM() {
        return 0;
    }

    /**
     * For angular velocity-controlled subsystem positions.
     *
     * @return The system's RPM.
     */
    default double getRPM(String key) {
        return 0;
    }
}
