package com.flarerobotics.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The main superclass interface for vision camera (hardware) interaction IO
 * classes.
 */
public interface VisionIO {

    /** The inputs class for the Vision Subsystem. */
    @AutoLog
    public static class VisionIOInputs {
        public boolean isCameraConnected = false;
        public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIDs = new int[0];
        public int bestTagID = -1;
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type) {}

    /**
     * An enum specifying which processing method is used when computing the pose.
     */
    public static enum PoseObservationType {
        MEGATAG_2, // Multi-tag with Limelight.
        PHOTONVISION // Multi-tag with PhotonVision when available, single tag otherwise.
    }

    /**
     * Updates the given vision inputs.
     *
     * @param inputs The inputs.
     */
    public default void updateInputs(VisionIOInputs inputs) {}
}
