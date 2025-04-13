package com.flarerobotics.lib.control;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

/** Utility class for basic kinematics related to wheel rotations. */
public class RotationalKinematics {
    private RotationalKinematics() {
        throw new UnsupportedOperationException("Cannot instantiate static class RotationalKinematics");
    }

    public static Distance distanceTravelledByWheel(Distance diameter, double reduction, double revs) {
        double circumferenceMeters = diameter.in(Meters) * Math.PI;
        double wheelRevs = revs / reduction;
        return Meters.of(circumferenceMeters * wheelRevs);
    }

    public static double motorRevsFromDistanceTravelledByWheel(
            Distance wheelDiameter, Distance travelled, double reduction) {
        double wheelRevs = travelled.in(Meters) / (wheelDiameter.in(Meters) * Math.PI);
        return wheelRevs * reduction;
    }
}
