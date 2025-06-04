# FlareLib for FRC

FlareLib-FRC is a robotics library designed for use in FIRST Robotics Competition (FRC) projects. It provides utilities, subsystems, and helpers to simplify robot development, including support for vision processing, localization, and simulation. The library integrates with WPILib and various vendor libraries to enhance functionality.

## Features

### Vision Processing
- A subsystem template for robot localization using AprilTags. Compatible with Limelight, PhotonVision, and PhotonVision simulation.
- Automatic AprilTag alignment via the `AlignToTagCommand2D` class.

### Geometry Utilities
- Provides utilities for working with WPILib's `Pose2d` and `Pose3d` classes, including translations and rotations.

### Simulation Support
- Integration with WPILib simulation tools, including `RoboRioSim` for simulating RoboRIO behavior.
- Custom subsystem simulation for cascade and continuous elevators along with an arm/pivot simulation.

### Autonomous Routines
- Includes an autonomous routine manager, which automatically handles the autonomous period.
- Allows for path generation from string, can be useful to fix autonomous conflicts right before a match.

### Logging and Diagnostics
- Integration with AdvantageKit for logging and diagnostics.
- Support for NT4 (NetworkTables 4) publishing and WPILOG file writing.

### Javadocs
- Includes Javadocs for readability and documentation.

## Getting Started

### Prerequisites
- Java 17
- WPILib 2025
- Gradle (included with WPILib projects)

### Installation and Deployment
1. Download the repository and extract it.
2. If you already have some code, you may add it to the `frc/robot` package.
3. Then, build your code while connected to the internet first to download the vendor libraries.
4. Finally, deploy your code into your robot.

## License
- This project is licensed under the GNU General Public License v3.0. See the LICENSE file for details.
