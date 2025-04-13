package com.flarerobotics.lib.vision.io;

import com.flarerobotics.lib.vision.LimelightHelpers;
import com.flarerobotics.lib.vision.VisionIO;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
    // Heading supplier as Rotation2d
    private final Supplier<Rotation2d> m_rotationSupplier;

    // NT Subscribers
    private final DoubleSubscriber m_latencySubscriber;
    private final DoubleArraySubscriber m_megatag2Subscriber;

    // The name of the Limelight
    private final String m_name;

    // Technical constant values for NT arrays
    private static final int kAverageTagDistIndex = 9;
    private static final int kTagCountIndex = 7;
    private static final int kLaterncyIndex = 6;
    private static final int kTagDataStartIndex = 11;
    private static final int kValsPerSample = 7;

    private static final double kLatencyTimeoutMs = 250;

    /**
     * Constructs a new VisionIOLimelight.
     *
     * @param name             The name of the Limelight.
     * @param rotationSupplier Supplier for the current robot rotation as reported
     *                         by the gyro/IMU, used for MegaTag2.
     */
    public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
        m_name = name;
        m_rotationSupplier = rotationSupplier;

        // NT Subscribers
        var table = NetworkTableInstance.getDefault().getTable(name);
        m_latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        m_megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the
        // given time range
        inputs.isCameraConnected =
                ((RobotController.getFPGATime() - m_latencySubscriber.getLastChange()) / 1000) < kLatencyTimeoutMs;

        // Update target observation
        inputs.latestTargetObservation = new TargetObservation(
                Rotation2d.fromDegrees(LimelightHelpers.getTX(m_name)),
                Rotation2d.fromDegrees(LimelightHelpers.getTY(m_name)));

        // Update orientation for MegaTag2
        double clampedHeading = MathUtil.inputModulus(m_rotationSupplier.get().getDegrees(), -180, 180);
        LimelightHelpers.SetRobotOrientation(m_name, clampedHeading, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Read new pose observations from NetworkTables
        Set<Integer> tagIDs = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();

        // Have to do this manually because in LimelightHelpers,
        // LimelightHelpers.getBotPose(String, String boolean) uses Pose2d instead of
        // Pose3d.
        for (var rawSample : m_megatag2Subscriber.readQueue()) {
            if (rawSample.value.length == 0) continue;
            for (int i = kTagDataStartIndex; i < rawSample.value.length; i += kValsPerSample) {
                tagIDs.add((int) rawSample.value[i]);
            }
            poseObservations.add(new PoseObservation(
                    rawSample.timestamp * 1e-6 - rawSample.value[kLaterncyIndex] * 1e-3, // Timestamp,
                    // based
                    // on
                    // server
                    // timestamp
                    // of
                    // publish
                    // and
                    // latency
                    LimelightHelpers.toPose3D(rawSample.value), // 3D pose estimate
                    0.0, // Ambiguity, zeroed because the pose is already disambiguated
                    (int) rawSample.value[kTagCountIndex], // Tag count
                    rawSample.value[kAverageTagDistIndex], // Average tag distance
                    PoseObservationType.MEGATAG_2 // Observation type
                    ));
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
