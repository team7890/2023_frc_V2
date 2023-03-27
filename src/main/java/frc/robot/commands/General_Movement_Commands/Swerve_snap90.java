// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.General_Movement_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Swerve_subsystem;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities;

public class Swerve_snap90 extends CommandBase {

  private Swerve_subsystem objSwerve;
  private double dYawAngle;
  private double dTargetAngle;
  private double dRotate;
  private double dDiffAngle;
  private double dKp = 0.25 / 45.0;    // make speed 0.25 if angle difference is 45 degrees

  /** Creates a new Swerve_snap90. */
  public Swerve_snap90(Swerve_subsystem objSwerve_in) {

    objSwerve = objSwerve_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // first get the current angle (getYawDouble is ranged to between 0 and 360 so it doesn't wrap)
    dYawAngle = objSwerve.getYawDouble();
    // now find closest 90 degree angle as the target
    if (dYawAngle > 45.0 && dYawAngle < 135.0) {dTargetAngle = 90.0;}
    if (dYawAngle > 135.0 && dYawAngle < 225.0) {dTargetAngle = 180.0;}
    if (dYawAngle > 225.0 && dYawAngle < 315.0) {dTargetAngle = 270.0;}
    if (dYawAngle > 315.0 && dYawAngle < 360.0) {dTargetAngle = 360.0;}   // be careful at 0 or 360 where it wraps
    if (dYawAngle > 0.0 && dYawAngle < 45.0) {dTargetAngle = 0.0;}        // be careful at wrap
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dRotate = 0.0;
    dYawAngle = objSwerve.getYawDouble();

    // correct for target angle if drive through disconuity at 0 or 360 degrees
    if (dTargetAngle == 360.0 && dYawAngle < 45.0) dTargetAngle = 0.0;
    if (dTargetAngle == 0.0 && dYawAngle > 315.0) dTargetAngle = 360.0;

    // calculate angle error diffangle
    dDiffAngle = dTargetAngle - dYawAngle;

    // if (dDiffAngle > 20.0) dRotate = 0.25;
    // else if (dDiffAngle > 1.0) dRotate = 0.1;
    // else if (dDiffAngle > -1.0) dRotate = 0.0;
    // else if (dDiffAngle > -20.0) dRotate = -0.1;
    // else dRotate = -0.25;

    dRotate = dDiffAngle * dKp;
    dRotate = dRotate + 0.07 * Math.signum(dRotate);   // make minimum dRotate 0.07 in whichever direction it needs to be
    dRotate = Utilities.limitVariable(-0.3, dRotate, 0.3);

     /* Drive */
     objSwerve.drive(
      new Translation2d(0.0, 0.0).times(Constants.Swerve.maxSpeed), 
      dRotate * Constants.Swerve.maxAngularVelocity, 
      true, 
      true
    );

    // SmartDashboard.putNumber("rotation", dRotate);
    // SmartDashboard.putNumber("diff angle", dDiffAngle);
    // SmartDashboard.putNumber("YawAngle", objSwerve.getYawDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objSwerve.drive(
      new Translation2d(0.0, 0.0).times(Constants.Swerve.maxSpeed), 
      0.0 * Constants.Swerve.maxAngularVelocity, 
      true, 
      true
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(dTargetAngle - dYawAngle) < 1.0;
  }
}
