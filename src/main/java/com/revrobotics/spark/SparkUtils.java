package com.revrobotics.spark;

import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkJNI;

/**
 * A static class to allow for for direct JNI SparkMax and SparkFlex interactions.
 */
public class SparkUtils {
	/** Constructor not avaiable. */
	private SparkUtils() {
		throw new UnsupportedOperationException("Cannot instantiate static class SparkUtils");
	}

	// Inversions //

	/**
	 * Sets wether the SparkMax is inverted or not.
	 *
	 * @param spark    The SparkMax.
	 * @param inverted True if inverted.
	 * @return The REVLibError code.
	 */
	public static REVLibError setInverted(SparkMax spark, boolean inverted) {
		return REVLibError.fromInt(CANSparkJNI.c_Spark_SetInverted(spark.sparkHandle, inverted));
	}

	/**
	 * Sets wether the SparkFlex is inverted or not.
	 *
	 * @param spark    The SparkFlex to use.
	 * @param inverted True if inverted.
	 * @return The REVLibError code.
	 */
	public static REVLibError setInverted(SparkFlex spark, boolean inverted) {
		return REVLibError.fromInt(CANSparkJNI.c_Spark_SetInverted(spark.sparkHandle, inverted));
	}

	/**
	 * Returns wether the SparkMax is inverted or not.
	 *
	 * @param spark The SparkMax.
	 * @return True if inverted.
	 */
	public static boolean getInverted(SparkMax spark) {
		return CANSparkJNI.c_Spark_GetInverted(spark.sparkHandle);
	}

	/**
	 * Returns wether the SparkFlex is inverted or not.
	 *
	 * @param spark The SparkFlex.
	 * @return True if inverted.
	 */
	public static boolean getInverted(SparkFlex spark) {
		return CANSparkJNI.c_Spark_GetInverted(spark.sparkHandle);
	}
}
