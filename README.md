# FlareLib for FRC

FlareLib is a lightweight Java library that builds upon WPILib to make robot software development more fast and convenient for FIRST Robotics Competition (FRC) teams. The library provides a set of subsystems, utilities, and simulation tools designed to:

* Enable off-robot testing, driver practice and robot tuning using realistic simulations.
* Provide implementations for common tasks such as vision-based alignment, kinematic computations, and autonomous routine management.

FlareLib lets teams focus on strategy and tuning rather than boilerplate functionality.

## Key Features

### Vision Processing

* **AprilTag Localization**: A vision subsystem template for reliable robot pose estimation using AprilTags, supporting both Limelight, PhotonVision hardware and simulations.
* **Automated Alignment**: Command-based `AlignToTagCommand2D` for centering and orienting the robot relative to detected fiducials.
* **Game Piece Detection**: A game piece detection subsystem template for locating and processing game pieces on the field. Supports both Limelight, PhotonVision hardware and simulations.

### Subsystem Presets
* **Bang-Bang Style Elevator**: A bang-bang controller style elevator, supporting the TalonFX, SparkMAX, and simulation.
* **Generic Rollers**: A subsystem template for any rollers that don't need precise speeds, supporting the TalonFX, SparkMAX and simulation.

### Simulation Support

* **AdvantageScope Subsystem Mocks**: Simulated AdvantageScope models for elevators (cascade-continuous) and arm/pivot assemblies.
* **Subsystem Simulations**: Simulated subsystems via `ElevatorSimTilted`, `CustomDCMotorSim` - mostly forks of WPILib simulations to include features such as current limiting.

### Autonomous Routines

* **Routine Scheduler**: A simple manager for sequencing and monitoring autonomous commands.
* **String-Based Routine Definitions**: Define trajectories via string patterns for rapid on-the-fly adjustments via dashboard inputs.

### Logging and Diagnostics

* **AdvantageKit Integration**: Built‑in hooks for recording system data via ShuffleBoard and AdvantageKit.
* **NetworkTables 4**: Telemetry streaming to external dashboards, such as ShuffleBoard, Elastic and SmartDashboard.

### Other Utilities
* Classes for geometry and vector math, shooter calculations, field region divisions, and more.

## Installation

### Prerequisites

* Java 17
* WPILib 2025
* Gradle (bundled with WPILib projects)

*Note: The following dependencies are pre-installed for FlareLib:*
* *AdvantageKit (v4.1.2)*
* *MapleSim (v0.3.11)*
* *PathPlanner (v2025.2.7)*
* *Phoenix 5 (v5.35.1) and 6 (v25.3.2)*
* *Photonlib (v2025.3.1)*
* *ReduxLib (v2025.0.1)*
* *REVLib (v2025.0.3)*
* *Studica (v2025.0.0)*
* *ThriftyLib (v2025.1.1)*
* *YAGSL (v2025.8.0)*


### Adding FlareLib to Your Project

1. Clone the repository to a directory:
   ```bash
   git clone https://github.com/aregulardev/flarelib-frc.git
   ```
2. Copy your dependencies, repositories, vendordeps to the cloned repository.
3. Move your code, classes, packages, etc. to the new repository after the dependencies.
4. Build and deploy using the standard WPILib tools.

**Note: Check for updates and fixes regularly and update the ID at the end in Step #2.**

## Usage Example

```java
import com.flarerobotics.lib.ascope.linear.ContinuousAscopeDisplay;
(...)

public class RobotContainer {
    (...)
    // AdvantageScope integration
    private final ContinuousAscopeDisplay m_display;

    public RobotContainer() {
        (...)
        // A perfectly upright 2-stage continuous elevator with a max height of 1.81m.
        m_display = new ContinuousAscopeDisplay("elevator", 2, 1.81, 0, m_elevator::getHeightMeters, () -> Rotation3d.kZero)
    }
}
```

## Javadocs
The Javadocs are accessible at https://aregulardev.github.io/flarelib-docs/.

## Quick Module Overview

### com.flarerobotics.lib
* **utils**: Includes generic utility classes such as `AllianceUtil` or `VisionUtils`.
* **subsystem**: Includes preset subsystems for systems such as rollers, LEDs, object detection, vision, etc.
* **sim**: Includes simulation models: `ElevatorSimTilted`, `CustomDCMotorSim`, `HardwareSimUtils`.
* **region**: Includes 2D and 3D field regions (mostly deprecated for external libraries).
* **math**: Includes math modules, such as `PhysicsUtil` and `BilinearInterpolator2D`.
* **control**: Includes classes related to control, such as `RotationalKinematics`, `ShooterCalculator`.
* **container**: Includes container classes, e.g. `Vector3` or `LoggedTunableNumber`.
* **auto**: Includes classes related to the autonomous period, such as `AutonomousManager` or `LineTrajectoryUtils`.
* **ascope**: Includes classes for displaying subsystems in AdvantageScope, such as `CascadeAscopeDisplay`.

### ctre.phoenix6.configs
* Includes a CTRE Talon utility class, `TalonUtils`.

### com.revrobotics.spark
* Includes a REV Spark utility class, `SparkUtils`.

## Contributing to FlareLib

Contributions are welcome under the following process:

1. Fork the repository and create a descriptive feature branch.
2. Adhere to WPILib coding standards and conventions, and project formatting (see [custom-formatter.xml](custom-formatter.xml)).
3. Submit a pull request with a clear summary of changes and any relevant test results.

## License

This project is licensed under the GNU General Public License v3.0. See [LICENSE](LICENSE) for the full text and details.

---

*Maintained by the Flare Robotics Software Team*
