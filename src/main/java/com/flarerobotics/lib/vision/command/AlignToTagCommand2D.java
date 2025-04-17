package com.flarerobotics.lib.vision.command;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.flarerobotics.lib.PoseUtils;
import com.flarerobotics.lib.container.PIDConstants;
import com.flarerobotics.lib.vision.VisionSubsystem;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Aligns the robot to the given tag from a birdseye 2D view.
 * 
 * <p>
 * Note: Uses <b>Meters</b> and <b>Radians</b> for PID values.
 * 
 * 
 * <p>
 * Example usage:
 * <p>
 * <code><pre>
 * Commands.defer(() -> new AlignToTagCommand2D( // Defer to always reconstruct the command
                            m_DriveSubsystem::getPose, // Field-relative pose
                            m_DriveSubsystem::driveFieldOriented, // Field-oriented drive
                            AlignToTagCommand2D.getClosestTagId(m_DriveSubsystem.getPose(), new int[] { // Get the nearest Reef tag
                                6, 7, 8, 9, 10, 11, // Red
                                17, 18, 19, 20, 21, 22 // Blue
                            }),
                            VisionConstants.kAlignSidewaysOffset, // Offset
                            VisionConstants.kAlignReefDistance, // Distance
                            0.9, // Chassis length 
                            new PIDConstants(3), // Translational PID
                            new PIDConstants(2, 0.01, 0.007), // Angular PID
                            m_DriveSubsystem), new HashSet<>() { // Command requirements
                                {
                                    add(m_DriveSubsystem);
                                }
                            }));
 * </pre></code>
 * 
 */
public class AlignToTagCommand2D extends Command {
    private Pose2d m_pose;
    private Supplier<Pose2d> m_supplier;

    private PIDController m_xPID, m_yPID, m_aPID;

    private Timer m_timer = new Timer();
    private double m_maxTimer = 5;

    private Consumer<ChassisSpeeds> m_consumer;

    private Supplier<Integer> m_tagId = () -> 0;

    private double m_alignDistanceMeters, m_horizontalOffsetMeters, m_txOffsetDegrees;

    /**
     * Constructs a new AlignToTagCommand2D.
     * 
     * @param poseSupplier           The field-relative robot pose supplier.
     * @param chassisSpeedsConsumer  The chassis speeds consumer.
     * @param tagId                  The tag ID to align to.
     * @param horizontalOffsetMeters The horizontal offset from the tag in meters.
     * @param alignDistanceMeters    The perpendicular offset from the tag in
     *                               meters. (Right is positive)
     * @param chassisLengthMeters    The length of the chassis when looked at from
     *                               the side while aligning in meters.
     * @param translationalConstants The translational PID constants.
     * @param angularConstants       The angular PID constants.
     * @param drive                  The drive subsystem for command requirements.
     */
    public AlignToTagCommand2D(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> chassisSpeedsConsumer, int tagId,
            double horizontalOffsetMeters, double alignDistanceMeters, double chassisLengthMeters,
            PIDConstants translationalConstants, PIDConstants angularConstants, Subsystem drive) {

        m_supplier = poseSupplier;
        m_consumer = chassisSpeedsConsumer;

        m_tagId = () -> tagId;

        m_alignDistanceMeters = alignDistanceMeters + chassisLengthMeters / 2;
        m_horizontalOffsetMeters = -horizontalOffsetMeters;

        m_txOffsetDegrees = 0;

        // Initialize the PID cntrollers
        m_xPID = new PIDController(translationalConstants.kP, translationalConstants.kI, translationalConstants.kD);
        m_yPID = new PIDController(translationalConstants.kP, translationalConstants.kI, translationalConstants.kD);
        m_aPID = new PIDController(angularConstants.kP, angularConstants.kI, angularConstants.kD);

        // Set the default tolerances
        m_xPID.setTolerance(Meters.convertFrom(0.5, Centimeters)); // 0.5 cm tolerance
        m_yPID.setTolerance(Meters.convertFrom(0.5, Centimeters)); // 0.5 cm tolerance
        m_aPID.setTolerance(Radians.convertFrom(1, Degrees)); // 1 degree tolerance

        // Limit angles
        m_aPID.enableContinuousInput(-180, 180);

        // Set the integral zones
        m_xPID.setIZone(translationalConstants.iZone);
        m_yPID.setIZone(translationalConstants.iZone);
        m_aPID.setIZone(angularConstants.iZone);

        // Timer initialization
        m_timer = new Timer();

        addRequirements(drive);
    }

    /**
     * Constructs a new AlignToTagCommand2D.
     * 
     * @param poseSupplier           The field-relative robot pose supplier.
     * @param chassisSpeedsConsumer  The chassis speeds consumer.
     * @param tagId                  The tag ID to align to.
     * @param horizontalOffsetMeters The horizontal offset from the tag in meters.
     * @param alignDistanceMeters    The perpendicular offset from the tag in
     *                               meters. (Right is positive)
     * @param txOffsetDegrees        The angular offset between the tag in degrees.
     * @param chassisLengthMeters    The length of the chassis when looked at from
     *                               the side while aligning in meters.
     * @param translationalConstants The translational PID constants.
     * @param angularConstants       The angular PID constants.
     * @param drive                  The drive subsystem for command requirements.
     */
    public AlignToTagCommand2D(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> chassisSpeedsConsumer, int tagId,
            double horizontalOffsetMeters, double alignDistanceMeters, double txOffsetDegrees,
            double chassisLengthMeters, PIDConstants translationalConstants, PIDConstants angularConstants,
            Subsystem drive) {
        m_supplier = poseSupplier;
        m_consumer = chassisSpeedsConsumer;

        m_tagId = () -> tagId;

        m_alignDistanceMeters = alignDistanceMeters + chassisLengthMeters / 2;
        m_horizontalOffsetMeters = -horizontalOffsetMeters;

        m_txOffsetDegrees = txOffsetDegrees;

        // Initialize the PID cntrollers
        m_xPID = new PIDController(translationalConstants.kP, translationalConstants.kI, translationalConstants.kD);
        m_yPID = new PIDController(translationalConstants.kP, translationalConstants.kI, translationalConstants.kD);
        m_aPID = new PIDController(angularConstants.kP, angularConstants.kI, angularConstants.kD);

        // Limit angles
        m_aPID.enableContinuousInput(-180, 180);

        // Set the default tolerances
        m_xPID.setTolerance(Meters.convertFrom(0.5, Centimeters)); // 0.5 cm tolerance
        m_yPID.setTolerance(Meters.convertFrom(0.5, Centimeters)); // 0.5 cm tolerance
        m_aPID.setTolerance(Radians.convertFrom(1, Degrees)); // 1 degree tolerance

        // Set the integral zones
        m_xPID.setIZone(translationalConstants.iZone);
        m_yPID.setIZone(translationalConstants.iZone);
        m_aPID.setIZone(angularConstants.iZone);

        // Timer initialization
        m_timer = new Timer();

        addRequirements(drive);
    }

    /**
     * Constructs a new AlignToTagCommand2D.
     * 
     * <p>
     * <i>Note: The supplier is only called once upon initialization.
     * 
     * @param poseSupplier           The field-relative robot pose supplier.
     * @param chassisSpeedsConsumer  The chassis speeds consumer.
     * @param tagId                  The supplier of the tag ID to align to.
     * @param horizontalOffsetMeters The horizontal offset from the tag in meters.
     * @param alignDistanceMeters    The perpendicular offset from the tag in
     *                               meters. (Right is positive)
     * @param chassisLengthMeters    The length of the chassis when looked at from
     *                               the side while aligning in meters.
     * @param translationalConstants The translational PID constants.
     * @param angularConstants       The angular PID constants.
     * @param drive                  The drive subsystem for command requirements.
     */
    public AlignToTagCommand2D(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> chassisSpeedsConsumer,
            Supplier<Integer> tagId,
            double horizontalOffsetMeters, double alignDistanceMeters, double chassisLengthMeters,
            PIDConstants translationalConstants, PIDConstants angularConstants, Subsystem drive) {

        m_supplier = poseSupplier;
        m_consumer = chassisSpeedsConsumer;

        m_tagId = tagId;

        m_alignDistanceMeters = alignDistanceMeters + chassisLengthMeters / 2;
        m_horizontalOffsetMeters = -horizontalOffsetMeters;

        m_txOffsetDegrees = 0;

        // Initialize the PID cntrollers
        m_xPID = new PIDController(translationalConstants.kP, translationalConstants.kI, translationalConstants.kD);
        m_yPID = new PIDController(translationalConstants.kP, translationalConstants.kI, translationalConstants.kD);
        m_aPID = new PIDController(angularConstants.kP, angularConstants.kI, angularConstants.kD);

        // Set the default tolerances
        m_xPID.setTolerance(Meters.convertFrom(0.5, Centimeters)); // 0.5 cm tolerance
        m_yPID.setTolerance(Meters.convertFrom(0.5, Centimeters)); // 0.5 cm tolerance
        m_aPID.setTolerance(Radians.convertFrom(1, Degrees)); // 1 degree tolerance

        // Limit angles
        m_aPID.enableContinuousInput(-180, 180);

        // Set the integral zones
        m_xPID.setIZone(translationalConstants.iZone);
        m_yPID.setIZone(translationalConstants.iZone);
        m_aPID.setIZone(angularConstants.iZone);

        // Timer initialization
        m_timer = new Timer();

        addRequirements(drive);
    }

    /**
     * Constructs a new AlignToTagCommand2D.
     * 
     * <p>
     * <i>Note: The supplier is only called once upon initialization.
     * 
     * @param poseSupplier           The field-relative robot pose supplier.
     * @param chassisSpeedsConsumer  The chassis speeds consumer.
     * @param tagId                  The supplier of the tag ID to align to.
     * @param horizontalOffsetMeters The horizontal offset from the tag in meters.
     * @param alignDistanceMeters    The perpendicular offset from the tag in
     *                               meters. (Right is positive)
     * @param txOffsetDegrees        The angular offset between the tag in degrees.
     * @param chassisLengthMeters    The length of the chassis when looked at from
     *                               the side while aligning in meters.
     * @param translationalConstants The translational PID constants.
     * @param angularConstants       The angular PID constants.
     * @param drive                  The drive subsystem for command requirements.
     */
    public AlignToTagCommand2D(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> chassisSpeedsConsumer,
            Supplier<Integer> tagId,
            double horizontalOffsetMeters, double alignDistanceMeters, double txOffsetDegrees,
            double chassisLengthMeters, PIDConstants translationalConstants, PIDConstants angularConstants,
            Subsystem drive) {
        m_supplier = poseSupplier;
        m_consumer = chassisSpeedsConsumer;

        m_tagId = tagId;

        m_alignDistanceMeters = alignDistanceMeters + chassisLengthMeters / 2;
        m_horizontalOffsetMeters = -horizontalOffsetMeters;

        m_txOffsetDegrees = txOffsetDegrees;

        // Initialize the PID cntrollers
        m_xPID = new PIDController(translationalConstants.kP, translationalConstants.kI, translationalConstants.kD);
        m_yPID = new PIDController(translationalConstants.kP, translationalConstants.kI, translationalConstants.kD);
        m_aPID = new PIDController(angularConstants.kP, angularConstants.kI, angularConstants.kD);

        // Limit angles
        m_aPID.enableContinuousInput(-180, 180);

        // Set the default tolerances
        m_xPID.setTolerance(Meters.convertFrom(0.5, Centimeters)); // 0.5 cm tolerance
        m_yPID.setTolerance(Meters.convertFrom(0.5, Centimeters)); // 0.5 cm tolerance
        m_aPID.setTolerance(Radians.convertFrom(1, Degrees)); // 1 degree tolerance

        // Set the integral zones
        m_xPID.setIZone(translationalConstants.iZone);
        m_yPID.setIZone(translationalConstants.iZone);
        m_aPID.setIZone(angularConstants.iZone);

        // Timer initialization
        m_timer = new Timer();

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        System.out.println("Starting AutoAlign...");
        var pose = VisionSubsystem.kLayout.getTagPose(m_tagId.get());
        if (!pose.isPresent()) {
            DriverStation.reportWarning("Unable to start AutoAlign2D: invalid tag ID", true);
            cancel();
            return;
        }
        // Offset the pose
        m_pose = PoseUtils.offsetSideways(PoseUtils.offsetPerpendicular(pose.get().toPose2d(), m_alignDistanceMeters),
                m_horizontalOffsetMeters);

        // Apply setpoints
        m_xPID.setSetpoint(m_pose.getX());
        m_yPID.setSetpoint(m_pose.getY());
        m_aPID.setSetpoint(180 + m_pose.getRotation().getDegrees() - m_txOffsetDegrees);

        // Reset the controllers
        m_xPID.reset();
        m_yPID.reset();
        m_aPID.reset();

        // Reset and start the timer
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        // Compute the velocities via the PID controllers
        Pose2d botPose = m_supplier.get();
        double vx = m_xPID.calculate(botPose.getX()), vy = m_yPID.calculate(botPose.getY()),
                va = m_aPID.calculate(botPose.getRotation().getDegrees());

        // Feed the velocities into the drivetrain
        m_consumer.accept(new ChassisSpeeds(vx, vy, Math.toRadians(va)));
    }

    @Override
    public boolean isFinished() {
        // End conditions:
        // * Timer exceeded max time
        // * Setpoint has been reached
        return m_xPID.atSetpoint() && m_yPID.atSetpoint()
                && m_aPID.atSetpoint() || m_timer.get() > m_maxTimer;
    }

    @Override
    public void end(boolean interrupted) {
        // Halt everything
        m_timer.stop();
        m_consumer.accept(new ChassisSpeeds(0, 0, 0));

        // Debug messages
        if (interrupted) {
            System.out.printf("AutoAlign has been interrupted. Time (s): %.2f\n", m_timer.get());
        } else {
            System.out.printf("AutoAlign has finished successfully. Time (s): %.2f\n", m_timer.get());
        }
    }

    /**
     * Sets the tolerance of the translational (x, y) PID controllers. Defaults to
     * 0.02 meters = 2 cm.
     * 
     * @param toleranceMeters The tolerance in meters.
     */
    public void setTranslationalTolerance(double toleranceMeters) {
        m_xPID.setTolerance(toleranceMeters);
        m_yPID.setTolerance(toleranceMeters);
    }

    /**
     * Sets the tolerance of the angular PID controller. Defaults to 0.03 rads ~=
     * 1.7 degs.
     * 
     * @param toleranceDegrees The tolerance in degrees.
     */
    public void setAngularTolerance(double toleranceDegrees) {
        m_aPID.setTolerance(Math.toRadians(toleranceDegrees));
    }

    /**
     * Sets the maximum duration of the timer. Defaults to 5 seconds.
     * 
     * @param maxSeconds The maximum duration in seconds.
     */
    public void setMaxTimer(double maxSeconds) {
        m_maxTimer = maxSeconds;
    }

    /**
     * Gets the closest tag ID to the given robot pose.
     * 
     * @param robotPose The field-relative robot pose.
     * @return The field-relative tag ID.
     */
    public static int getClosestTagId(Pose2d robotPose) {
        int id = 0;
        double minDist = Double.POSITIVE_INFINITY;
        for (AprilTag tag : VisionSubsystem.kLayout.getTags()) {
            double distance = tag.pose.toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            if (distance < minDist) {
                id = tag.ID;
                minDist = distance;
            }
        }

        return id;
    }

    /**
     * Gets the closest tag ID to the given robot pose in account to the valid tag
     * IDs.
     * 
     * @param robotPose       The field-relative robot pose.
     * @param validTagsFilter The tag IDs to choose from.
     * @return The field-relative tag ID.
     */
    public static int getClosestTagId(Pose2d robotPose, int[] validTagsFilter) {
        int id = 0;
        double minDist = Double.POSITIVE_INFINITY;
        for (AprilTag tag : VisionSubsystem.kLayout.getTags()) {
            double distance = tag.pose.toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            if (distance < minDist
                    && Arrays.binarySearch(validTagsFilter, tag.ID) != -1) {
                id = tag.ID;
                minDist = distance;
            }
        }

        return id;
    }
}
