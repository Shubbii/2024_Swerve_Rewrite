// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * FRC Team 2168 example swerve drivetrain code, updated with new WPILib classes to simplify math and
 * rely less on external unupdated libraries.
 * 
 * Drivetrain is configured with modules in the order of:
 * front left (FL), front right (FR), back left (BL), and back right (BR)
 */
public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private static Drivetrain instance;

  // configuration settings
  private static final int WHEEL_COUNT = 4;
  private static final double MAX_WHEEL_SPEED = 10.0; // meters/sec
  private final double MAX_ROTATION_SPEED = rotationsToRad(3.0); // in radians
  private static final double DRIVEBASE_LENGTH = 0.9; // in meters, distance from the center of a front wheel to the closest back wheel center
  private static final double DRIVEBASE_WIDTH = 0.9; // in meters, distance from the centers of a left wheel to the closest right wheel
  private SwerveDriveKinematics swerveConfig;
  private final double[] ENCODER_OFFSET = {0.0, 0.0, 0.0, 0.0}; // encoder offsets should be set to azimuth position readings when modules are at their zeroed positions

  // motors
  private TalonFX[] driveTalons = new TalonFX[WHEEL_COUNT];
  private TalonFX[] azimuthTalons = new TalonFX[WHEEL_COUNT];

  // gyro
  private Pigeon2 gyro;

  // wpilib-based driving and logging
  private SwerveModuleState[] moduleStates = new SwerveModuleState[WHEEL_COUNT];
  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[WHEEL_COUNT];
  private static final double GEAR_RATIO = 5.60; // varies based on module type used
  private static final double WHEEL_CIRCUMFERENCE_IN = 4.0 * Math.PI;
  private static final double WHEEL_CIRCUMFERENCE_M = WHEEL_CIRCUMFERENCE_IN * 0.0254;

  // inverts (should vary based on physical module configuration)
  private final boolean[] DRIVE_INVERT = {true, false, true, false}; 
  private final SensorDirectionValue[] ENCODER_INVERT = {SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive, 
    SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive};

  // current limits
  private final boolean ENABLE_DRIVE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_DRIVE_CURRENT_LIMIT = 35.0; // amps
  private final double TRIGGER_DRIVE_THRESHOLD_LIMIT = 40.0; // amps
  private final double TRIGGER_DRIVE_THRESHOLD_TIME = 0.2; // seconds

  private final boolean ENABLE_AZIMUTH_CURRENT_LIMIT = true;
  private final double CONTINUOUS_AZIMUTH_CURRENT_LIMIT = 10.0; // amps
  private final double TRIGGER_AZIMUTH_THRESHOLD_LIMIT = 10.0; // amps
  private final double TRIGGER_AZIMUTH_THRESHOLD_TIME = 0.2; // seconds

  // odometry
  private SwerveDriveOdometry odometry;

  public static enum DriveState { // designates motor control mode which should be used for driving the wheels.
    PERCENTOUT,                   // percent output is generally recommended for joysticks, while velocity will give better auto functionality
    VELOCITY
  }

  public Drivetrain() {
    swerveConfig = new SwerveDriveKinematics(                   // module positions relative to center of drivetrain
      new Translation2d(DRIVEBASE_LENGTH/2, DRIVEBASE_WIDTH/2), // positive x values travel towards the front of the robot
      new Translation2d(DRIVEBASE_LENGTH/2, -DRIVEBASE_WIDTH/2),// positive y values travel towards the left of the robot
      new Translation2d(-DRIVEBASE_LENGTH/2, DRIVEBASE_WIDTH/2),// positions should be put in order of FL -> FR -> BL -> BR
      new Translation2d(-DRIVEBASE_LENGTH/2, -DRIVEBASE_WIDTH/2)
    );

    configureModules();

    gyro = new Pigeon2(Constants.CANDevices.PIGEON_ID, "rio");
    gyro.setYaw(0.0); // reset heading upon robot initialization

    resetDriveEncoders();
    odometry = new SwerveDriveOdometry(swerveConfig, Rotation2d.fromDegrees(gyro.getYaw().getValue()), modulePositions);
  }

  /**
   * @return an instance of the Drivetrain subsystem
   */
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }

    return instance;
  }

  /**
   * configures module devices (motors and encoders) with the correct settings
   */
  private void configureModules() {
    Slot0Configs driveSlot0Config = new Slot0Configs();
    Slot0Configs azimuthSlot0Config = new Slot0Configs();

    MagnetSensorConfigs azimuthEncoderMagnetConfig = new MagnetSensorConfigs();

    MotorOutputConfigs driveOutputConfig = new MotorOutputConfigs();
    MotorOutputConfigs azimuthOutputConfig = new MotorOutputConfigs();

    CurrentLimitsConfigs driveCurrentConfig = new CurrentLimitsConfigs();
    CurrentLimitsConfigs azimuthCurrentConfig = new CurrentLimitsConfigs();

    FeedbackConfigs driveFeedbackConfig = new FeedbackConfigs();
    FeedbackConfigs azimuthFeedbackConfig = new FeedbackConfigs();

    MotionMagicConfigs driveMotionMagicConfig = new MotionMagicConfigs();
    MotionMagicConfigs azimuthMotionMagicConfig = new MotionMagicConfigs();

    azimuthCurrentConfig.withSupplyCurrentLimitEnable(ENABLE_AZIMUTH_CURRENT_LIMIT)
    .withSupplyCurrentLimit(CONTINUOUS_AZIMUTH_CURRENT_LIMIT)
    .withSupplyCurrentThreshold(TRIGGER_AZIMUTH_THRESHOLD_LIMIT)
    .withSupplyTimeThreshold(TRIGGER_AZIMUTH_THRESHOLD_TIME);

    driveCurrentConfig.withSupplyCurrentLimitEnable(ENABLE_DRIVE_CURRENT_LIMIT)
    .withSupplyCurrentLimit(CONTINUOUS_DRIVE_CURRENT_LIMIT)
    .withSupplyCurrentThreshold(TRIGGER_DRIVE_THRESHOLD_LIMIT)
    .withSupplyTimeThreshold(TRIGGER_DRIVE_THRESHOLD_TIME);

    azimuthFeedbackConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
    azimuthSlot0Config.withKP(5.0) // tune pid gains as needed
    .withKI(1.0)
    .withKD(0.0)
    .withKV(0.0)
    .withKA(0.0)
    .withKS(0.0);

    azimuthMotionMagicConfig.withMotionMagicAcceleration(150.0); // rotations/sec^2
    azimuthMotionMagicConfig.withMotionMagicCruiseVelocity(40.0); // rotations/sec

    driveFeedbackConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    driveSlot0Config.withKP(0.3) // tune pid gains as needed
    .withKI(0.7)
    .withKD(0.0)
    .withKV(0.0)
    .withKA(0.0)
    .withKS(0.0);

    driveMotionMagicConfig.withMotionMagicAcceleration(30); // rotations/sec^2
    driveMotionMagicConfig.withMotionMagicCruiseVelocity(15); // rotations/sec

    for (int i = 0; i < WHEEL_COUNT; i++) {
      azimuthEncoderMagnetConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
      azimuthEncoderMagnetConfig.withMagnetOffset(-ENCODER_OFFSET[i]);
      azimuthEncoderMagnetConfig.withSensorDirection(ENCODER_INVERT[i]);

      CANcoder azimuthEncoder = new CANcoder(Constants.CANDevices.CANCODER_IDS[i], "rio");
      azimuthEncoder.getConfigurator().apply(azimuthEncoderMagnetConfig);
      azimuthEncoder.close();
      azimuthFeedbackConfig.withFeedbackRemoteSensorID(Constants.CANDevices.CANCODER_IDS[i]);

      TalonFX azimuthTalon = new TalonFX(Constants.CANDevices.AZIMUTH_MOTORS[i], "rio");
      TalonFXConfigurator azimuthConfiguration = azimuthTalon.getConfigurator();

      azimuthConfiguration.apply(new TalonFXConfiguration());
      azimuthConfiguration.apply(azimuthSlot0Config);
      azimuthConfiguration.apply(azimuthFeedbackConfig);
      azimuthConfiguration.apply(azimuthMotionMagicConfig);
      azimuthConfiguration.apply(azimuthOutputConfig);
      azimuthConfiguration.apply(azimuthCurrentConfig);
      azimuthTalon.setInverted(true);
      azimuthTalon.setNeutralMode(NeutralModeValue.Brake);
      azimuthTalon.close();

      TalonFX driveTalon = new TalonFX(Constants.CANDevices.DRIVE_MOTORS[i], "rio");
      TalonFXConfigurator driveConfiguration = driveTalon.getConfigurator();

      driveConfiguration.apply(new TalonFXConfiguration());
      driveConfiguration.apply(driveSlot0Config);
      driveConfiguration.apply(driveFeedbackConfig);
      driveConfiguration.apply(driveMotionMagicConfig);
      driveConfiguration.apply(driveOutputConfig);
      driveConfiguration.apply(driveCurrentConfig);
      driveTalon.setInverted(DRIVE_INVERT[i]);
      driveTalon.setNeutralMode(NeutralModeValue.Brake);
      driveTalon.close();

      modulePositions[i] = new SwerveModulePosition(0.0, new Rotation2d(0.0));
      moduleStates[i] = new SwerveModuleState(0.0, new Rotation2d(0.0));
    }
  }

  /**
   * drives swerve modules by converting a ChassisSpeeds object into individual swerve module states
   * @param fieldRelativeSpeeds chassis speed to be converted
   * @param driveState whether to interpret speed units as percent output or a unit of velocity
   */
  public void drive(ChassisSpeeds fieldRelativeSpeeds, DriveState driveState) {
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, gyro.getRotation2d());
    moduleStates = swerveConfig.toSwerveModuleStates(robotRelativeSpeeds);

    for (int i = 0; i < WHEEL_COUNT; i++) {
      SwerveModuleState desiredState = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromRotations(getAzimuthPosition(i)));

      switch(driveState) {
        case PERCENTOUT:
          double percentOutput = MathUtil.clamp(desiredState.speedMetersPerSecond/MAX_WHEEL_SPEED, -1.0, 1.0); // prevents wheel speed from exceeding limit
          driveTalons[i].set(percentOutput);
          break;
        case VELOCITY:
          double velocity = MathUtil.clamp(desiredState.speedMetersPerSecond, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED); // prevents wheel speed from exceeding limit
          driveTalons[i].setControl(new VelocityVoltage(velocity));
      }
      azimuthTalons[i].setControl(new MotionMagicVoltage(desiredState.angle.getRotations()));
    }
  }

  /**
   * drives robot during autos and pathing commands
   * @param robotRelativeSpeeds robot relative speeds to set the drivetrain to
   */
  public void driveAuto(ChassisSpeeds robotRelativeSpeeds) {
    drive(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, gyro.getRotation2d()), DriveState.VELOCITY);
  }

  /**
   * @return yaw heading of pigeon 2 gyro (ccw+)
   */
  public double getHeading() {
    return gyro.getYaw().getValue();
  }

  /**
   * set gyro yaw to angle input
   * @param angle heading to be set in degrees
   */
  public void setHeading(double angle) {
    gyro.setYaw(angle);
  }

  /**
   * resets heading to zero
   */
  public void resetHeading() {
    gyro.setYaw(0.0);
  }

  public void resetDriveEncoders() {
    for (int i = 0; i < WHEEL_COUNT; i++) {
      driveTalons[i].setPosition(0.0);
    }
  }

  /**
   * gets the max wheel speed of the swerve module, used to determine speed output values during percent out drive mode
   * @return maximum wheel speed
   */
  public double getMaxWheelSpeed() {
    return MAX_WHEEL_SPEED;
  }

  /**
   * gets the max rotational speed of the swerve module, used to determine speed output values during percent out drive mode
   */
  public double getMaxRotationalSpeed() {
    return MAX_ROTATION_SPEED;
  }

  /**
   * @return amount of meters travelled by a module
   * @param moduleID specifies which module from 0-3 (fl, fr, bl, or br)
   */
  public double getModuleDrivenMeters(int moduleID) {
    return rotationsToMeters(driveTalons[moduleID].getRotorPosition().getValue());
  }

  /**
   * @return absolute encoder position of azimuth motor (in rotations)
   * @param moduleID specifies which azimuth to return position from (0 - 3), (fl, fr, bl, br)
   */
  public double getAzimuthPosition(int moduleID) {
    return azimuthTalons[moduleID].getPosition().getValue();
  }

  /**
   * uses gear ratio and wheel circumference to convert meters to rotations
   * @param meters unit input
   * @return equivalent distance in rotations
   */
  public double metersToRotations(double meters) {
    return (meters * GEAR_RATIO) * WHEEL_CIRCUMFERENCE_M;
  }
  
  /**
   * uses gear ratio and wheel circumference to convert rotations to meters
   * @param rotations unit input
   * @return equivalent distance in meters
   */
  public double rotationsToMeters(double rotations) {
    return (rotations / GEAR_RATIO) / WHEEL_CIRCUMFERENCE_M;
  }

  /**
   * unit conversion of encoder rotations to radians
   * @param rotations unit input
   * @return equivalent angle in radians
   */
  public double rotationsToRad(double rotations) {
    return (rotations) * 2.0 * Math.PI;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < WHEEL_COUNT; i++) {
      modulePositions[i] = new SwerveModulePosition(getModuleDrivenMeters(i), 
      Rotation2d.fromRotations(azimuthTalons[i].getPosition().getValue()));
    }
    odometry.update(gyro.getRotation2d(), modulePositions);
  }
}
