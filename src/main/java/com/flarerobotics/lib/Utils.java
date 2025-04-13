package com.flarerobotics.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * A static utility class for generic functions.
 */
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
     * @return true if roughly equal.
     */
    public static boolean epsilonEquals(double a, double b) {
        return MathUtil.isNear(a, b, 1e-7);
    }

    /**
     * Rotates the current pose around a point in 3D space.
     *
     * @param point The point in 3D space to rotate around.
     * @param rot   The rotation to rotate the pose by.
     * @return The new rotated pose.
     */
    public static Pose3d rotateAround(Pose3d currentPose, Translation3d point, Rotation3d rot) {
        return new Pose3d(currentPose.getTranslation().rotateAround(point, rot),
                currentPose.getRotation().rotateBy(rot));
    }
}
