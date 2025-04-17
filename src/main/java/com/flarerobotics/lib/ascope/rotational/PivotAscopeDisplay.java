package com.flarerobotics.lib.ascope.rotational;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates an arm / pivot and calculates joint positions, publishing
 * them to Network Tables (via Akit Logger) for visualization in AdvantageScope.
 *
 * <p>
 * Note: Use {@link #PivotAscopeDisplay(String, Supplier, double, Supplier)} with
 * length = 0 for a pivot display.
 */
public class PivotAscopeDisplay {
    private final String m_name;

    // Number of joints, can be extended
    private final int m_jointCount;

    // Segment lengths for each arm segment (e.g., upper arm, forearm)
    private final double[] m_segmentLengths;

    // Suppliers for joint angles (assuming they are given in radians)
    private final Supplier<Rotation3d[]> m_jointAngles;

    // If the arm extends (telescopic arm), a supplier for extension
    private final Supplier<Double[]> m_extensionSupplier;

    // Base pose of the arm relative to the robot
    private final Supplier<Pose3d> m_basePose;

    /**
     * Constructs a new PivotAscopeDisplay.
     *
     * @param name                 The name of the system.
     * @param basePose             The pose supplier for the root of the arm.
     * @param segmentLengthsMeters The length of each joint.
     * @param jointAnglesRads      The supplier for the joint angles, in radians.
     * @param extensionSupplier    The extensions supplier in meters, used for
     *                             telescoping / extending arms.
     */
    public PivotAscopeDisplay(
            String name,
            Supplier<Pose3d> basePose,
            double[] segmentLengthsMeters,
            Supplier<Rotation3d[]> jointAnglesRads,
            Supplier<Double[]> extensionSupplier) {
        /*
         * if (segmentLengthsMeters.length != jointAnglesRads.length) {
         * throw new
         * IllegalArgumentException("Segment lengths and joint angles arrays must have the same length"
         * );
         * }
         */

        m_name = name;
        m_jointCount = segmentLengthsMeters.length;
        m_segmentLengths = segmentLengthsMeters;
        m_jointAngles = jointAnglesRads;
        m_extensionSupplier = extensionSupplier;
        m_basePose = basePose;
    }

    /**
     * Constructs a single-jointed new PivotAscopeDisplay.
     *
     * @param name              The name of the system.
     * @param basePose          The pose supplier for the root of the arm.
     * @param length            The length of the arm in meters.
     * @param angleSupplier     The supplier for the arm angle, in radians.
     * @param extensionSupplier The extensions supplier in meters, used for
     *                          telescoping / extending arms.
     */
    public PivotAscopeDisplay(
            String name,
            Supplier<Pose3d> basePose,
            double length,
            Supplier<Rotation3d> angleSupplier,
            Supplier<Double> extensionSupplier) {
        this(name, basePose, new double[] {length}, () -> new Rotation3d[] {angleSupplier.get()}, () ->
                new Double[] {extensionSupplier.get()});
    }

    /**
     * Constructs a single-jointed new PivotAscopeDisplay.
     *
     * <p>
     * Note: Use length = 0 for pivot.
     *
     * @param name          The name of the system.
     * @param basePose      The pose supplier for the root of the arm.
     * @param length        The length of the arm in meters.
     * @param angleSupplier The supplier for the arm angle, in radians.
     */
    public PivotAscopeDisplay(
            String name, Supplier<Pose3d> basePose, double length, Supplier<Rotation3d> angleSupplier) {
        this(name, basePose, new double[] {length}, () -> new Rotation3d[] {angleSupplier.get()}, () ->
                new Double[] {0.0});
    }

    /**
     * Computes the final pose of the arm's end effector using forward kinematics.
     *
     * @return The pose of the end effector.
     */
    public Pose3d computeEndEffectorPose() {
        if (m_extensionSupplier.get().length != m_jointAngles.get().length
                || m_extensionSupplier.get().length != m_jointCount
                || m_jointAngles.get().length != m_jointCount) {
            throw new IllegalArgumentException("Invalid length parameters provided to PivotAscopeDisplay");
        }

        // Start with the base pose
        Pose3d pose = m_basePose.get();

        // Chain each joint's transformation
        for (int i = 0; i < m_jointCount; i++) {
            // Translate along the segment length
            double length = m_segmentLengths[i];

            // For telescopic arms, you might incorporate the extension supplier on a
            // specific joint
            if (m_extensionSupplier != null) {
                length += m_extensionSupplier.get()[i];
            }

            // Create a translation for this segment
            Transform3d translation = new Transform3d(new Translation3d(0, 0, length), Rotation3d.kZero);

            // Get the rotation for this joint
            Rotation3d jointRotation = m_jointAngles.get()[i];
            Transform3d rotation = new Transform3d(new Translation3d(), jointRotation);

            // Chain the transformations
            pose = pose.transformBy(rotation).transformBy(translation);
        }

        return pose;
    }

    /**
     * Returns the pose of the specified joint (1-indexed).
     * Joint 1 is the first joint (i.e. the transformation after the base, including
     * the first segment's rotation and translation).
     *
     * @param joint  The joint number. (1-indexed)
     * @param offset The offset applied to the final position.
     * @return The pose of that joint, computed using forward kinematics.
     * @throws IllegalArgumentException if the joint number is invalid.
     */
    public Pose3d getJointPose(int joint, Transform3d offset) {
        if (m_extensionSupplier.get().length != m_jointAngles.get().length
                || m_extensionSupplier.get().length != m_jointCount
                || m_jointAngles.get().length != m_jointCount) {
            throw new IllegalArgumentException("Invalid length parameters provided to PivotAscopeDisplay");
        }

        if (joint < 1 || joint > m_jointCount) {
            throw new IllegalArgumentException("Invalid joint number " + joint);
        }

        // Start with the base pose.
        Pose3d pose = m_basePose.get();

        // Chain transformations for joints 1 up to the specified joint.
        // Note: array indexing is 0-indexed.
        for (int i = 0; i < joint; i++) {
            double length = m_segmentLengths[i];
            // For telescopic arms, incorporate extension for the first joint (if
            // applicable)
            if (i == 0 && m_extensionSupplier != null) {
                length += m_extensionSupplier.get()[i];
            }

            // First apply the joint rotation.
            Rotation3d jointRotation = m_jointAngles.get()[i];
            Transform3d rotationTransform = new Transform3d(new Translation3d(), jointRotation);

            // Then apply the translation along the segment.
            Transform3d translationTransform = new Transform3d(new Translation3d(0, 0, length), Rotation3d.kZero);

            // Update the cumulative pose.
            pose = pose.transformBy(rotationTransform).transformBy(translationTransform);
        }

        return pose.transformBy(offset);
    }

    /**
     * Returns the pose of the specified joint (1-indexed).
     * Joint 1 is the first joint (i.e. the transformation after the base, including
     * the first segment's rotation and translation).
     *
     * @param joint  The joint number. (1-indexed)
     * @return The pose of that joint, computed using forward kinematics.
     * @throws IllegalArgumentException if the joint number is invalid.
     */
    public Pose3d getJointPose(int joint) {
        return getJointPose(joint, new Transform3d());
    }

    /**
     * Updates the Network Table outputs. Data is stored in the path as a Pose3d:
     * <p>
     * <code>Simulation/{SYSTEM_NAME}Arm/Joint{I}/Pose</code>
     *
     * @param offset The offset to apply to each joint.
     */
    public void update(Transform3d offset) {
        for (int j = 1; j <= m_jointCount; j++) {
            Logger.recordOutput("Simulation/" + m_name + "Arm/Joint" + j + "/Pose", getJointPose(j, offset));
        }
    }

    /**
     * Updates the Network Table outputs. Data is stored in the path as a Pose3d:
     * <p>
     * <code>Simulation/{SYSTEM_NAME}Arm/Joint{I}/Pose</code>
     *
     * @param offsets The offsets to apply to each joint.
     */
    public void update(Transform3d[] offsets) {
        for (int j = 1; j <= m_jointCount; j++) {
            Logger.recordOutput("Simulation/" + m_name + "Arm/Joint" + j + "/Pose", getJointPose(j, offsets[j - 1]));
        }
    }

    /**
     * Updates the Network Table outputs. Data is stored in the path as a Pose3d:
     * <p>
     * <code>Simulation/{SYSTEM_NAME}Arm/Joint{I}/Pose</code>
     */
    public void update() {
        for (int j = 1; j <= m_jointCount; j++) {
            Logger.recordOutput("Simulation/" + m_name + "Arm/Joint" + j + "/Pose", getJointPose(j));
        }
    }

    /**
     * Attaches a mechanism to end of the arm.
     *
     * @param offset The offset from the end, uses a supplier because attached
     *               mechanisms generally move or rotate. Generally used to supply
     *               rotational offsets.
     * @return The pose supplier.
     */
    public Supplier<Pose3d> attachMechanism(Supplier<Transform3d> offset) {
        return () -> computeEndEffectorPose().transformBy(offset.get());
    }
}
