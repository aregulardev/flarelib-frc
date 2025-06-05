package com.flarerobotics.lib.vision;

import static edu.wpi.first.units.Units.Meters;

import com.flarerobotics.lib.region.RectangularRegion;
import com.flarerobotics.lib.vision.VisionIO.PoseObservationType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// Code modified from the AdvantageKit vision template.

/**
 * A vision subsystem used for robot localization via AprilTags. Supports
 * Limelight, Photon and Photon Simulation.
 */
public class VisionSubsystem extends SubsystemBase {
    private final VisionConsumer m_addMesaurement;
    private final VisionIO[] m_cameras;
    private final VisionIOInputsAutoLogged[] m_inputs;
    private final Alert[] m_disconnectedAlerts;

    public static final AprilTagFieldLayout kLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final RectangularRegion kFieldRegion = new RectangularRegion(
            new Translation2d(0.0, 0.0),
            new Translation2d(kLayout.getFieldLength(), kLayout.getFieldWidth())); // 2025 Field Size

    // Subsystem configuration variables
    private double[] m_cameraStdDevFactors;
    // Unit: Meters (m)
    private double m_maxHeight = 1, m_maxAmbiguity = 0.3;
    // Standard deviation baselines, for 1 meter distance and 1 tag.
    // Is adjusted automatically based on distance and the number of tags.
    // Units: Meters (m), Radians (rads) respectively.
    private double m_linearStdDevBaseline = 0.02, m_angularStdDevBaseline = 0.06;

    // MT2 factors
    private double m_MT2_linearStdDevFactor = 0.5, m_MT2_angularStdDevFactor = Double.POSITIVE_INFINITY;

    /**
     * Constructs a new VisionSubsystem.
     *
     * @param consumer The consumer to add a vision mesaurement. Recommended to use
     *                 a WPILib pose estimator.
     * @param io       The IO interfaces to provide for each individual camera.
     */
    public VisionSubsystem(VisionConsumer addMesaurement, VisionIO... cameras) {
        m_addMesaurement = addMesaurement;
        m_cameras = cameras;

        m_cameraStdDevFactors = new double[cameras.length];

        // Initialize inputs
        m_inputs = new VisionIOInputsAutoLogged[cameras.length];
        for (int i = 0; i < m_inputs.length; i++) {
            m_inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts for each camera
        m_disconnectedAlerts = new Alert[cameras.length];
        for (int i = 0; i < m_inputs.length; i++) {
            m_disconnectedAlerts[i] = new Alert(
                    "Vision camera with index '" + Integer.toString(i) + "'' is disconnected.", AlertType.kWarning);
        }
    }

    @Override
    public void periodic() {
        // Process inputs for each camera
        for (int i = 0; i < m_cameras.length; i++) {
            m_cameras[i].updateInputs(m_inputs[i]);
            Logger.processInputs("Vision/CameraData/Camera_i" + Integer.toString(i), m_inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Iterate over each camera
        for (int cameraIndex = 0; cameraIndex < m_cameras.length; cameraIndex++) {
            // Update disconnected alert
            m_disconnectedAlerts[cameraIndex].set(!m_inputs[cameraIndex].isCameraConnected);

            // Initialize camera-specific logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : m_inputs[cameraIndex].tagIDs) {
                var tagPose = kLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : m_inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose mesaurement
                boolean rejectPose = observation.tagCount() == 0 // Visible tag check
                        || (observation.tagCount() == 1 && observation.ambiguity() > m_maxAmbiguity) // Ambiguity
                        // threshold
                        // with 1 tag
                        || Math.abs(observation.pose().getZ()) > m_maxHeight // Height limit threshold
                        || !kFieldRegion.isPoseWithinArea(observation.pose().toPose2d()); // Field boundary check

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                    continue; // Skip if rejected
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Calculate standard deviations
                // f_stdDev = d^2 / n
                // => directly proportional with d^2 (from the inverse square law)
                // => inversely proportional with n (precision increases with the amt. of tags)
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = m_linearStdDevBaseline * stdDevFactor;
                double angularStdDev = m_angularStdDevBaseline * stdDevFactor;

                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= m_MT2_linearStdDevFactor;
                    angularStdDev *= m_MT2_angularStdDevFactor;
                }
                if (cameraIndex < m_cameraStdDevFactors.length) {
                    linearStdDev *= m_cameraStdDevFactors[cameraIndex];
                    angularStdDev *= m_cameraStdDevFactors[cameraIndex];
                }

                // Push vision observation
                m_addMesaurement.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)); // in the format of [x, y, theta]
            }

            // Log camera data
            Logger.recordOutput(
                    "Vision/CameraData/Camera_i" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/CameraData/Camera_i" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/CameraData/Camera_i" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/CameraData/Camera_i" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing
     * with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTX(int cameraIndex) {
        return m_inputs[cameraIndex].latestTargetObservation.tx();
    }

    /**
     * Returns the Y angle to the best target, which can be used for simple servoing
     * with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTY(int cameraIndex) {
        return m_inputs[cameraIndex].latestTargetObservation.ty();
    }

    /**
     * Returns wether the vision detects a valid target.
     *
     * @return True if a valid target is detected.
     */
    public boolean getTV() {
        for (int i = 0; i < m_inputs.length; i++) {
            if (m_inputs[i].tagIDs.length != 0) return true;
        }
        return false;
    }

    /**
     * Returns wether the vision detects a valid target.
     *
     * @param cameraIndex The index of the camera.
     * @return True if a valid target is detected.
     */
    public boolean getTV(int cameraIndex) {
        return m_inputs[cameraIndex].tagIDs.length != 0;
    }

    /**
     * Returns the cameras as an IO interface array.
     *
     * @return The camera array.
     */
    public VisionIO[] getCamerasIO() {
        return m_cameras;
    }

    /**
     * Returns the camera inputs.
     *
     * @return The input array.
     */
    public VisionIOInputsAutoLogged[] getCameraInputs() {
        return m_inputs;
    }

    /**
     * Sets the height threshold (invalidation point) for the vision pose. Defaults
     * to 1 meter.
     *
     * @param height The max height.
     */
    public void setMaxHeight(Distance height) {
        m_maxHeight = height.in(Meters);
    }

    /**
     * Sets the maximum ambiguity for the robot's pose. Defaults to 0.3.
     *
     * @param max The maximum ambiguity.
     */
    public void setMaxAmbiguity(double max) {
        m_maxAmbiguity = max;
    }

    /**
     * Sets the baseline for the linear standard deviations. Defaults to 0.02.
     *
     * <p>
     * Standard deviations are for 1 meter distance and 1 tag. Is adjusted
     * automatically based on distance and the number of tags.
     *
     * @param baseline The baseline.
     */
    public void setLinearStdDevBaseline(double baseline) {
        m_linearStdDevBaseline = baseline;
    }

    /**
     * Sets the baseline for the angular standard deviations. Defaults to 0.06.
     *
     * <p>
     * Standard deviations are for 1 meter distance and 1 tag. Is adjusted
     * automatically based on distance and the number of tags.
     *
     * @param baseline The baseline.
     */
    public void setAngularStdDevBaseline(double baseline) {
        m_angularStdDevBaseline = baseline;
    }

    /**
     * Sets the linear deviations factor applied for MegaTag2. Defaults to 0.5.
     *
     * @param factor The factor.
     */
    public void setLinearStdDevFactor_MT2(double factor) {
        m_MT2_linearStdDevFactor = factor;
    }

    /**
     * Sets the angular standard deviations factor applied for MegaTag2. Defaults to
     * Infinity.
     *
     * @param factor The factor.
     */
    public void setAngularStdDevFactor_MT2(double factor) {
        m_MT2_angularStdDevFactor = factor;
    }

    /**
     * Sets the standard deviation factors for each camera in the same order it was
     * provided in the constructor.
     *
     * @param factors The factors.
     */
    public void setCameraStdDevFactors(double[] factors) {
        m_cameraStdDevFactors = factors;
    }

    /**
     * The consumer functional interface for the <i>addVisionMesaurement</i> method.
     */
    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
