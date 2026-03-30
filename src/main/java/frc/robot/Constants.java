// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimatorConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class rioCan {
    public static CANBus rio = new CANBus("rio");
  }

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
  }

  public static class FieldConstants {

    public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Translation2d blueHub =
        new Translation2d(
            tagLayout.getTagPose(26).get().getX() + Units.inchesToMeters(47.0) / 2.0,
            tagLayout.getFieldWidth() / 2.0);

    public static final Translation2d redHub =
        blueHub.rotateAround(
            new Translation2d(tagLayout.getFieldLength() / 2.0, tagLayout.getFieldWidth() / 2.0),
            Rotation2d.k180deg);

    public static final Translation2d blueFerryBottom = new Translation2d(2, 2);
    public static final Translation2d blueFerryTop =
        new Translation2d(2, tagLayout.getFieldWidth() - blueFerryBottom.getY());

    public static final Translation2d redFerryBottom =
        new Translation2d(
            tagLayout.getFieldLength() - blueFerryBottom.getX(), blueFerryBottom.getY());
    public static final Translation2d redFerryTop =
        new Translation2d(redFerryBottom.getX(), blueFerryTop.getY());

    public static final double ferryXThresholdBlue = 5;
    public static final double ferryXThresholdRed =
        tagLayout.getFieldLength() - ferryXThresholdBlue;

    public static final double ferryYThreshold = tagLayout.getFieldWidth() / 2.0;

    public static final Rectangle2d blueBumpZone = new Rectangle2d(Pose2d.kZero, 1, 1);
    public static final Rectangle2d redBumpZone = new Rectangle2d(Pose2d.kZero, 1, 1);

    // uncomment if using the test tag layout
    // public static final AprilTagFieldLayout tagLayout;
    // static {
    //   try {
    //     tagLayout =
    //         new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/test-tag-layout.json");
    //   } catch (Exception e) {
    //     throw new RuntimeException(e);
    //   }
    // }
  }

  public static class VisionConstants {

    public static final double singleTagStdDevsScaler = 5;

    public static final double ambiguityThreshold = 0.2;

    public static final double xBoundMargin = 0.01;
    public static final double yBoundMargin = 0.01;
    public static final double zBoundMargin = 0.1;

    public static final String leftArducamName = "left-arducam";
    public static final String rightArducamName = "right-arducam";

    public static final VisionPoseEstimatorConstants leftArducam =
        new VisionPoseEstimatorConstants(
            leftArducamName,
            new Transform3d(
                -Units.inchesToMeters(9.333),
                Units.inchesToMeters(9.508),
                Units.inchesToMeters(18.087),
                new Rotation3d(0, 0, Units.degreesToRadians(23))),
            0.1,
            3,
            7);

    public static final VisionPoseEstimatorConstants rightArducam =
        new VisionPoseEstimatorConstants(
            rightArducamName,
            new Transform3d(
                -Units.inchesToMeters(9.333),
                -Units.inchesToMeters(9.508),
                Units.inchesToMeters(18.087),
                new Rotation3d(0, 0, -Units.degreesToRadians(23))),
            0.1,
            3,
            7);
  }
}
