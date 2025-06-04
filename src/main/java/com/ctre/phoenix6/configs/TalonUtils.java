package com.ctre.phoenix6.configs;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * A generic utility class for TalonFX and TalonFXS.
 */
public class TalonUtils {

    private TalonUtils() {
    }

    /**
     * Gets the current configuration of the given TalonFX.
     * 
     * @param fx the TalonFX to get the configuration from.
     * @return The current configuration of the TalonFX.
     */
    public static TalonFXConfiguration getTalonFXConfiguration(TalonFX fx) {
        /*
         * Background Code:
         * TalonFXConfiguration config = new TalonFXConfiguration();
         * StringBuilder serializedString = new StringBuilder();
         * StatusCode err = fx.getConfigurator().getConfigsPrivate(serializedString, 2);
         * if (err == StatusCode.OK) {
         * config.deserialize(serializedString.toString());
         * }
         * fx.getConfigurator().getConfigsPrivate(null, 0);
         */

        TalonFXConfiguration config = new TalonFXConfiguration();
        StatusCode status = fx.getConfigurator().refresh(config);
        if (status != StatusCode.OK) {
            DriverStation.reportWarning("Failed to refresh TalonFX configuration: " + status, true);
        }

        return config;
    }

    /**
     * Gets the current configuration of the given TalonFXS.
     * 
     * @param fxs the TalonFXS to get the configuration from.
     * @return The current configuration of the TalonFXS.
     */
    public static TalonFXSConfiguration getTalonFXSConfiguration(TalonFXS fxs) {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        StatusCode status = fxs.getConfigurator().refresh(config);
        if (status != StatusCode.OK) {
            DriverStation.reportWarning("Failed to refresh TalonFXS configuration: " + status, true);
        }

        return config;
    }
}
