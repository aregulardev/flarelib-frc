package com.flarerobotics.lib.utils;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Utilities assosciated with alliance color. */
public class AllianceUtil {
	private AllianceUtil() {}

	/**
	 * Flips the given blue-relative pose according to the current alliance.
	 *
	 * @param blueRelativePose The blue-alliance relative pose.
	 * @return The appropriately flipped pose.
	 */
	public static Pose2d flipWithAlliance(Pose2d blueRelativePose) {
		return isRedAlliance() ? FlippingUtil.flipFieldPose(blueRelativePose) : blueRelativePose;
	}

	/**
	 * Returns whether the current alliance is red.
	 *
	 * @return Whether the current alliance is red.
	 */
	public static boolean isRedAlliance() {
		var opt = DriverStation.getAlliance();

		return opt.isPresent() && opt.get() == Alliance.Red;
	}
}
