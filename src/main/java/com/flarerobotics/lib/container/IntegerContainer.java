package com.flarerobotics.lib.container;

/**
 * A container for an integer value, used for passing by reference into a lambda expression.
 * Used to avoid errors.
 */
public class IntegerContainer {
	public int m_value;

	/**
	 * Constructs a new IntegerContainer.
	 *
	 * @param value The value to store.
	 */
	public IntegerContainer(int value) {
		m_value = value;
	}
}
