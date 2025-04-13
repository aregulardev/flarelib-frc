package com.flarerobotics.lib.container;

import lombok.Getter;
import lombok.Setter;

/** A 3D vector class. */
@Getter
@Setter
public class Vector3 {
    public final double x;
    public final double y;
    public final double z;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // Returns a new vector that is a copy of this one.
    public Vector3 copy() {
        return new Vector3(x, y, z);
    }

    // Adds another vector to this one and returns a new vector.
    public Vector3 add(Vector3 other) {
        return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    // Substract another vector from this one and returns a new vector.
    public Vector3 sub(Vector3 other) {
        return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
    }

    // Scales this vector by a scalar and returns a new vector.
    public Vector3 scale(double scalar) {
        return new Vector3(this.x * scalar, this.y * scalar, this.z * scalar);
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }
}
