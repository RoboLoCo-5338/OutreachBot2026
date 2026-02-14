package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;

public class VisionConstants {

  public static AprilTagFieldLayout aprilTagLayout;

  static {
    try {
      aprilTagLayout =
          new AprilTagFieldLayout(
              // once we have the april tag field layout it can be added here in place of
              // placeholder.json
              new File(Filesystem.getDeployDirectory(), "field_map_feb_14_15_50_59.json").toPath());
    } catch (IOException e) {
      throw new RuntimeException("Failed to load AprilTag layout", e);
    }
  }

  // Camera names
  // These are also placeholders until we find a place to put the cameras
  // there will be 2 cameras on everybot
  public static String poseCamera0Name = "camera_0";
  public static String poseCamera1Name = "camera_1";

  // Robot to camera transforms
  // the values for the poses and stuff are just placeholder values. These numbers are not the
  // actual
  // things so dont ignore it
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(10.5),
          -Units.inchesToMeters(15.55),
          Units.inchesToMeters(8.5),
          new Rotation3d(0.0, -Units.degreesToRadians(16.0), 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(18.5),
          Units.inchesToMeters(6.30),
          Units.inchesToMeters(12.5),
          new Rotation3d(0.0, Units.degreesToRadians(30.0), 0.0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0 / 2, // Camera 0
        1.0 / 2 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static double objectDetectionConfidence = 0.78;
  public static double objectDetectionAmbiguity = 0.5;
}
