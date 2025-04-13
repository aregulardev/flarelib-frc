package com.flarerobotics.lib.container;

/**
 * A container for a double value, used for passing by reference into a lambda
 * expression. Used to avoid errors.
 */
public class DoubleContainer {
    public double m_value;

    public DoubleContainer(double value) {
        m_value = value;
    }
}
