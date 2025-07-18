package com.flarerobotics.lib.subsystem.vision.io;

import com.flarerobotics.lib.subsystem.vision.LibVisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for simulation using PhotonVision's built-in simulator. */
public class LibVisionIOPhotonVisionSim extends LibVisionIOPhotonVision {
	private static VisionSystemSim m_visionSim;

	private final Supplier<Pose2d> m_poseSupplier;
	private final PhotonCameraSim m_cameraSim;

	/**
	 * Constructs a new LibVisionIOPhotonVisionSim.
	 *
	 * @param name         The name of the camera.
	 * @param poseSupplier Supplier for the robot pose to use in simulation.
	 */
	public LibVisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
		super(name, robotToCamera);
		m_poseSupplier = poseSupplier;

		// Initialize simulation (if not already created)
		if (m_visionSim == null) {
			m_visionSim = new VisionSystemSim("main");
			m_visionSim.addAprilTags(LibVisionSubsystem.kLayout);
		}

		// Adds the simulation camera
		var cameraProperties = new SimCameraProperties();
		m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties, LibVisionSubsystem.kLayout);
		m_visionSim.addCamera(m_cameraSim, robotToCamera);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		m_visionSim.update(m_poseSupplier.get());
		super.updateInputs(inputs);
	}
}
