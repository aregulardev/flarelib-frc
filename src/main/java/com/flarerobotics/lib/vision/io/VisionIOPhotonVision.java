package com.flarerobotics.lib.vision.io;

import com.flarerobotics.lib.vision.VisionIO;
import com.flarerobotics.lib.vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    public final PhotonCamera m_camera;
    public final Transform3d m_robotToCamera;

    /**
     * Constructs a new VisionIOPhotonVision.
     *
     * @param name          The name of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot (the
     *                      offset from the center).
     */
    public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
        m_camera = new PhotonCamera(name);
        m_robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update connection status
        inputs.isCameraConnected = m_camera.isConnected();

        // Read new camera observations
        Set<Short> tagIDs = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var result : m_camera.getAllUnreadResults()) {
            // Update latest target observation
            if (result.hasTargets()) {
                inputs.latestTargetObservation = new TargetObservation(
                        Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                        Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
            } else {
                inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
            }

            // Add pose observation
            if (result.multitagResult.isPresent()) { // Multitag result (if possible)
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(m_robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate total tag distance for an average
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance +=
                            target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add tag IDs
                tagIDs.addAll(multitagResult.fiducialIDsUsed);

                // Add observation
                poseObservations.add(new PoseObservation(
                        result.getTimestampSeconds(), // Timestamp
                        robotPose, // 3D pose estimate
                        multitagResult.estimatedPose.ambiguity, // Pose ambiguity
                        multitagResult.fiducialIDsUsed.size(), // Tag count
                        totalTagDistance / result.targets.size(), // Average tag distance
                        PoseObservationType.PHOTONVISION)); // Observation type

            } else if (!result.targets.isEmpty()) { // Single tag result
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = VisionSubsystem.kLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    // Compute the field-relative robot pose
                    Transform3d fieldToTarget = new Transform3d(
                            tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(m_robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
                    // Add tag ID
                    tagIDs.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(new PoseObservation(
                            result.getTimestampSeconds(), // Timestamp
                            robotPose, // 3D pose estimate
                            target.poseAmbiguity, // Pose ambiguity
                            1, // Tag count
                            cameraToTarget.getTranslation().getNorm(), // Average tag distance
                            PoseObservationType.PHOTONVISION)); // Observation type
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tagIDs = new int[tagIDs.size()];
        int i = 0;
        for (int id : tagIDs) {
            inputs.tagIDs[i++] = id;
        }
    }
}
