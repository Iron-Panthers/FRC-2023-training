// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.util.MacUtil.IS_COMP_BOT;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.Drive.Dims;
import frc.robot.subsystems.NetworkWatchdogSubsystem.IPv4;
import frc.robot.subsystems.RGBSubsystem.RGBColor;
import java.util.List;

@SuppressWarnings("java:S1118")
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Drive {
    // max voltage delivered to drivebase
    // supposedly useful to limit speed for testing
    public static final double MAX_VOLTAGE = 12.0;
    // maximum velocity
    // FIXME measure this value experimentally
    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        6380.0 // falcon 500 free speed rpm
            / 60.0
            * SdsModuleConfigurations.MK4_L2.getDriveReduction()
            * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
            * Math.PI;
    // theoretical value
    // FIXME measure and validate experimentally
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
        MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0)
            * .5;

    /** the maximum amount of angular error pid loops will tolerate for rotation */
    public static final double ANGULAR_ERROR = 1.0;
    /** the minimum magnitude of the right stick for it to be used as a new rotation angle */
    public static final double ROTATE_VECTOR_MAGNITUDE = .7;

    public static final class Dims {
      // FIXME validate with hardware
      public static final double TRACKWIDTH_METERS =
          .5207; // 20.5 inches (source: cad) converted to meters
      public static final double WHEELBASE_METERS = TRACKWIDTH_METERS; // robot is square

      public static final double BUMPER_WIDTH_METERS = .851;
    }

    /*
     module layout:
        ┌──────
     ┌─►│#   ##steer motor
     │  │  ##cancoder
     │  │##drive motor
     module number

     steer is always left
     from corner perspective

     robot visualization:
    ┌──────────────────────┐
    │2   10          04   1│
    │  25              24  │
    │11     S      D     03│
    │     D          S     │
    │                      │
    │                      │
    │     S          D     │
    │       D      S       │
    │12    ┌────────┐    02│
    │  26  │        │  27  │
    │3   13│  batt  │01   4│
    └──────┴───┬┬───┴──────┘
               ││
               ││
               ▼▼
         software front
     */

    public static final class Modules {
      public static final class FrontRight { // Module 1
        public static final int DRIVE_MOTOR = 4;
        public static final int STEER_MOTOR = 3;
        public static final int STEER_ENCODER = 24;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(8.07400 + 180) // comp bot offset
                : -Math.toRadians(129.375 + 180); // practice bot offset
      }

      public static final class FrontLeft { // Module 2
        public static final int DRIVE_MOTOR = 11;
        public static final int STEER_MOTOR = 10;
        public static final int STEER_ENCODER = 25;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(274.562 + 180) // comp bot offset
                : -Math.toRadians(129.375 + 180); // practice bot offset
      }

      public static final class BackLeft { // Module 3
        public static final int DRIVE_MOTOR = 13;
        public static final int STEER_MOTOR = 12;
        public static final int STEER_ENCODER = 26;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(225.082 + 180) // comp bot offset
                : -Math.toRadians(307.793 + 180); // practice bot offset
      }

      public static final class BackRight { // Module 4
        public static final int DRIVE_MOTOR = 2;
        public static final int STEER_MOTOR = 1;
        public static final int STEER_ENCODER = 27;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(335.124 + 180) // comp bot offset
                : -Math.toRadians(241.963 + 180); // practice bot offset
      }
    }
  }

  public static final class Vision {

    public static final class FrontCam {
      public static final String NAME = "frontCam";
      public static final Transform3d ROBOT_TO_CAM =
          new Transform3d(
              // 9.867 in to the right looking from behind the front of the robot
              // 7 inch forward from center
              // up 17.422 inches
              new Translation3d(
                  0.1778, // front/back
                  0.2506218, // left/right
                  0.4425188 // up/down
                  ),
              new Rotation3d(
                  0,
                  Math.toRadians(-11.5), // angle up/down
                  0));
    }

    public static final class BackCam {
      public static final String NAME = "backCam";
      public static final Transform3d ROBOT_TO_CAM =
          new Transform3d(
              // 9.867 in to the right looking from behind the front of the robot
              // 48.5 inches up
              // two inches forward
              new Translation3d(
                  0.0508, // front/back
                  -0.2506218, // left/right
                  1.2319 // up/down
                  ),
              new Rotation3d(0, Math.toRadians(17), Math.PI));
    }
  }

  public static final class PoseEstimator {
    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS =
        Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                0.1, // x
                0.1, // y
                0.1 // theta
                );

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global
     * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in
     * meters and radians.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                .9, // x
                .9, // y
                .9 * Math.PI // theta
                );

    public static final double CAMERA_CAPTURE_LATENCY_FUDGE_MS = 11;

    /** about one inch */
    public static final double DRIVE_TO_POSE_XY_ERROR_MARGIN_METERS = .05;

    public static final double DRIVE_TO_POSE_THETA_ERROR_MARGIN_DEGREES = 2;
  }

  public static final class NetworkWatchdog {
    /** The IP addresses to ping for testing bridging, on the second vlan. */
    public static final List<IPv4> TEST_IP_ADDRESSES =
        List.of(IPv4.internal(17), IPv4.internal(18), IPv4.internal(19));

    /**
     * The number of ms (sleep delta using oshi system uptime) to wait before beginning to ping the
     * test IP.
     */
    public static final int BOOT_SCAN_DELAY_MS = 50_000;

    /** The number of seconds for ping to wait before giving up on reaching a device. */
    public static final int PING_TIMEOUT_SECONDS = 2;

    /** The number of ms to wait before retrying successful health checks. */
    public static final int HEALTHY_CHECK_INTERVAL_MS = 5_000;

    /**
     * The number of ms to leave the switching pdh port off before turning it back on as part of
     * rebooting the network switch.
     */
    public static final int REBOOT_DURATION_MS = 1_000;

    /**
     * The number of ms to wait before rerunning health checks after a failed check which triggered
     * switch reboot.
     */
    public static final int SWITCH_POWERCYCLE_SCAN_DELAY_MS = 25_000;
  }

  public static final class Lights {
    public static final int CANDLE_ID = 34;
    public static final int NUM_LEDS =
        91
            // 8 inside the candle
            + 8;

    public static final class Colors {
      public static final RGBColor YELLOW = new RGBColor(255, 107, 0);
      public static final RGBColor PURPLE = new RGBColor(127, 0, 127);
      public static final RGBColor RED = new RGBColor(255, 0, 0);
      public static final RGBColor BLUE = new RGBColor(0, 0, 255);
      public static final RGBColor PINK = new RGBColor(250, 35, 100);
      public static final RGBColor MINT = new RGBColor(55, 255, 50);
      public static final RGBColor TEAL = new RGBColor(0, 255, 255);
    }
  }
}
