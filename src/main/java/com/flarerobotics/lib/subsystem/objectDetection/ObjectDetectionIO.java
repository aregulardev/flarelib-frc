package com.flarerobotics.lib.subsystem.objectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The main superclass interface for object detection camera (hardware) interaction IO classes.
 */
public interface ObjectDetectionIO {

	/** The inputs class for the Object Detection Subsystem. */
	@AutoLog
	public static class ObjectDetectionIOInputs {
		public boolean isCameraConnected = false;
		public GamePieceObservation[] targetObservations = new GamePieceObservation[0];
	}

	/** Represents the data of a targeted game piece. */
	public static record GamePieceObservation(Rotation2d tx, Rotation2d ty, double distance, Pose2d fieldPose,
			double classIndex, double measurementTimestamp, boolean isTipped, double aspectRatio) {}

	/**
	 * Updates the given vision inputs.
	 *
	 * @param inputs The inputs.
	 */
	public default void updateInputs(ObjectDetectionIOInputs inputs) {}

	/**
	 * Sets the current pipeline index.
	 *
	 * @param pipelineIndex The index.
	 */
	public default void setPipeline(int pipelineIndex) {};
}
