// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.OI;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Drivetrain.DriveState;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithJoystick extends Command {
  /** Creates a new DriveWithJoystick. */
  Drivetrain drivetrain;
  OI oi;
  double kDriveInvert = 1.0;
  double chassisRot = 0.0;
  ChassisSpeeds drivingSpeeds;
  public DriveWithJoystick(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = OI.getInstance();

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      kDriveInvert = -1.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (OI.joystickChooser.getSelected() == "flight") {
      if (oi.driverJoystick.isPressedButtonA()) {
        chassisRot = -1.0 * Math.PI;
      }
      else if (oi.driverJoystick.isPressedButtonB()) {
        chassisRot = 1.0 * Math.PI;
      }
      else {
        chassisRot = 0.0;
      }
    }
    else {
      chassisRot = -oi.getLimitedDriverJoystickZValue() * drivetrain.getMaxRotationalSpeed();
    }
    drivingSpeeds = new ChassisSpeeds(oi.getLimitedDriverJoystickYValue() * drivetrain.getMaxTranslationSpeed() * kDriveInvert,
    -oi.getLimitedDriverJoystickXValue() * drivetrain.getMaxTranslationSpeed() * kDriveInvert, // set to negative as positive x on joystick is opposite direction of positive x in chassis speeds object
    chassisRot); // set to negative as positive z on joystick is opposite direction of positive z in chassis speeds object

    drivetrain.drive(drivingSpeeds, DriveState.PERCENTOUT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0), DriveState.PERCENTOUT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
