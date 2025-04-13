package com.flarerobotics.lib;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import java.util.ArrayList;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * A static battery simulation class to update the WPILib's {@link BatterySim}
 * class.
 */
public class BatteryUpdater {
    private static ArrayList<Supplier<Double>> m_currents = new ArrayList<>();

    @Getter
    private static double m_nominalVoltage = 12.5;

    @Getter
    private static double m_batteryResistanceOhms = 0.02;

    /**
     * Registers a current drawer (an object that draws current), such as a
     * mechanism or subsystem.
     *
     * @param currentSupplier The current supplier.
     */
    public static void registerCurrentDrawer(Supplier<Double> currentSupplier) {
        m_currents.add(currentSupplier);
    }

    /** Updates the battery simulation. Call periodically to update. */
    public static void update() {
        double[] currents = new double[m_currents.size()];
        for (int i = 0; i < m_currents.size(); i++) {
            currents[i] = m_currents.get(i).get();
        }

        BatterySim.calculateLoadedBatteryVoltage(m_nominalVoltage, m_batteryResistanceOhms, currents);
    }

    /**
     * Sets the nominal voltage of the battery. Defaults to 12.5V.
     *
     * @param nominalVoltage The voltage.
     */
    public static void setNominalVoltage(double nominalVoltage) {
        m_nominalVoltage = nominalVoltage;
    }

    /**
     * Sets the resistance. Defaults to 0.02.
     *
     * @param resistanceOhms The resistance in Ohms.
     */
    public static void setBatteryResistance(double resistanceOhms) {
        m_batteryResistanceOhms = resistanceOhms;
    }
}
