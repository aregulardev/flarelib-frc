package com.flarerobotics.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;

/** A static utility class for generic functions. */
public class Utils {
    /** Unconstructuble as Utils is a static class. */
    private Utils() {
        throw new UnsupportedOperationException("Cannot instantiate static class Utils");
    }

    /**
     * Returns wether a number is close to another number by a negligible amount
     * (10^-7).
     *
     * @param a The first number.
     * @param b The second number.
     * @return True if roughly equal.
     */
    public static boolean epsilonEquals(double a, double b) {
        return MathUtil.isNear(a, b, 1e-7);
    }

    /**
     * Disables the "command loop overrun" warning.
     *
     * @param robot The main "Robot.java" class.
     * @return True if successful.
     */
    public static boolean disableLoopOverrunWarnings(IterativeRobotBase robot) {
        try {
            Field field = IterativeRobotBase.class.getField("m_watchdog");
            field.setAccessible(true);
            ((Watchdog) field.get(robot)).disable();
            System.out.println("Command loop overrun warnings have been disabled.");
            return true;
        } catch (Exception ex) {
            DriverStation.reportWarning(
                    "Unable to disable command loop overrun warnings with exception: " + ex.getStackTrace(), false);
            return false;
        }
    }

    /**
     * Sets the duration before a "command loop overrun" warning is thrown.
     *
     * @param robot   The main "Robot.java" class.
     * @param timeout The timeout in seconds.
     * @return True if successful.
     */
    public static boolean setLoopOverrunTimeout(IterativeRobotBase robot, double timeout) {
        try {
            Field field = IterativeRobotBase.class.getField("m_watchdog");
            field.setAccessible(true);
            ((Watchdog) field.get(robot)).setTimeout(timeout);
            CommandScheduler.getInstance().setPeriod(timeout);
            System.out.println("Command loop overrun warnings have been set to a timeout of " + timeout + ".");
            return true;
        } catch (Exception ex) {
            DriverStation.reportWarning(
                    "Unable to set command loop overrun warning timeouts with exception: " + ex.getStackTrace(), false);
            return false;
        }
    }
}
