package com.flarerobotics.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A number which can be tuned in SmartDashboard. For any numbers that need to be tuned during
 * runtime.
 *
 * <p>
 * <b>Can be used for</b>: PID tuning, elevator height tuning, ...
 */
public class TunableNumber {
	private String m_key;
	private double m_defaultValue;

	public static boolean doTuning;

	/**
	 * Constructs a new TunableNumber.
	 *
	 * @param key The key on SmartDashboard.
	 */
	public TunableNumber(String key) {
		m_key = key;
	}

	/**
	 * If tuning mode is enabled, returns the tuned number. Otherwise returns the default value.
	 *
	 * @return The current value.
	 */
	public double get() {
		return doTuning ? SmartDashboard.getNumber(m_key, m_defaultValue) : m_defaultValue;
	}

	/**
	 * Returns the default value.
	 *
	 * @return The default value.
	 */
	public double getDefault() { return m_defaultValue; }

	/**
	 * Sets the default value of the number.
	 *
	 * @param defaultValue The default value.
	 */
	public void setDefault(double defaultValue) {
		m_defaultValue = defaultValue;

		if (doTuning) SmartDashboard.putNumber(m_key, SmartDashboard.getNumber(m_key, defaultValue));
	}
}
