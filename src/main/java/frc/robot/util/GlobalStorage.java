package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A global storage for storing robot-specific variables, allowing for easy data transfer
 * between subsystems.
 */
public class GlobalStorage {
	private static final Map<String, Object> list = Collections.synchronizedMap(new HashMap<>());

	/**
	 * Returns the value assosciated with the key.
	 *
	 * @param <T> The type of the value.
	 * @param key The key to obtain the value from.
	 * @return The obtained key. Null if an error is thrown.
	 */
	@SuppressWarnings("unchecked")
	public static <T> T get(RobotKey key) {
		try {
			return (T) list.get(key.keyName);
		} catch (Exception e) {
			DriverStation.reportWarning("GlobalStorage exception in method get(key): " + e.getMessage(), true);
			return null;
		}
	}

	/**
	 * Sets the value assosciated with the key.
	 *
	 * @param <T>   The type of the value.
	 * @param key   The key to set the value of.
	 * @param value The new value to set.
	 */
	public static <T> void set(RobotKey key, T value) {
		try {
			if (list.get(key.keyName) != null) list.replace(key.keyName, value);
			else list.put(key.keyName, value);
		} catch (Exception e) {
			DriverStation.reportWarning("GlobalStorage exception in method set(key, value): " + e.getMessage(), true);
		}
	}

	/**
	 * Removes and returns the value assosciated with the key.
	 *
	 * @param <T> The type of the value.
	 * @param key The key to remove.
	 * @return The removed value, null if not present. Null if an error is thrown.
	 */
	@SuppressWarnings("unchecked")
	public static <T> T remove(RobotKey key) {
		try {
			return (T) list.remove(key.keyName);
		} catch (Exception e) {
			DriverStation.reportWarning("GlobalStorage exception in method remove(key): " + e.getMessage(), true);
			return null;
		}
	}

	/** The keys for {@link GlobalStorage}. */
	public enum RobotKey {
		kAutoRangeSpeeds("AutoRange_Speeds", KeyCategory.kAdditionalChassisSpeeds),
		kAssistToGamePieceSpeeds("AssistGamePiece_Speeds", KeyCategory.kAdditionalChassisSpeeds);

		public final String keyName;
		public final KeyCategory category;

		RobotKey(String keyName, KeyCategory category) {
			this.keyName = keyName;
			this.category = category;
		}

		/**
		 * Returns all RobotKey instances with the given category.
		 *
		 * @param category The category.
		 * @return The obtained keys.
		 */
		public static ArrayList<RobotKey> getKeysFromCategory(KeyCategory category) {
			ArrayList<RobotKey> keys = new ArrayList<>();
			for (var k : RobotKey.values())
				if (k.category.equals(category)) keys.add(k);
			return keys;
		}

		/**
		 * Returns the values of each key in the given category.
		 *
		 * @param <T>      The type of values in the category, all must be of the same type.
		 * @param category The category of keys to check from.
		 * @return The obtained list of values, empty if an exception is caught.
		 */
		@SuppressWarnings("unchecked")
		public static <T> List<T> getValuesFromCategory(KeyCategory category) {
			try {
				return getKeysFromCategory(category).stream().map(key -> (T) GlobalStorage.get(key)).toList();
			} catch (Exception e) {
				DriverStation.reportWarning(
						"GlobalStorage RobotKey exception in getValuesFromCategory(category): " + e.getMessage(), true);
				return new ArrayList<>();
			}
		}
	}

	/** An enum showing what category each key belongs in. */
	public enum KeyCategory {
		kAdditionalChassisSpeeds;
	}
}
