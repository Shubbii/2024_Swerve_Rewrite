// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /**
   * IDs for all CAN Devices, including swerve module motors and encoders, are initialized in this class.
   */
  public static class CANDevices {
    public static final int DRIVE_FL = 2;
    public static final int DRIVE_FR = 13;
    public static final int DRIVE_BL = 12;
    public static final int DRIVE_BR = 3;

    public static final int AZIMUTH_FL = 11;
    public static final int AZIMUTH_FR = 1;
    public static final int AZIMUTH_BL = 0;
    public static final int AZIMUTH_BR = 6;

    public static final int CANCODER_FL = 14;
    public static final int CANCODER_FR = 4;
    public static final int CANCODER_BL = 8;
    public static final int CANCODER_BR = 7;

    public static final int[] DRIVE_MOTORS = {DRIVE_FL, DRIVE_FR, DRIVE_BL, DRIVE_BR};
    public static final int[] AZIMUTH_MOTORS = {AZIMUTH_FL, AZIMUTH_FR, AZIMUTH_BL, AZIMUTH_BR};
    public static final int[] CANCODER_IDS = {CANCODER_FL, CANCODER_FR, CANCODER_BL, CANCODER_BR};

    public static final int PIGEON_ID = 17;
  }

  public static class Controllers {
    public static final int DRIVER_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;
    public static final int TEST_JOYSTICK = 5;
  }
}
