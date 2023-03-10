// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve_subsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve_balance extends CommandBase {
  private Swerve_subsystem s_Swerve;
  private double dTrans;
  // private double dTrans_remember;
  private double dStrafe;
  private double dRotate;
  private double dKp;
  private boolean bRobotCentric;
  private boolean bOnRamp;
  private double dYaw;

  /** Creates a new AutoSwerve. */
  public Swerve_balance(Swerve_subsystem s_Swerve_in, double dTrans_in, double dStrafe_in, double dRotate_in, boolean bRobotCentric_in) {
    s_Swerve = s_Swerve_in;
    dTrans = dTrans_in;
    // dTrans_remember = dTrans_in;
    dStrafe = dStrafe_in;
    dRotate = dRotate_in;
    bRobotCentric = bRobotCentric_in;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // dTrans = dTrans_remember;
    dStrafe = 0.0;
    dRotate = 0.0;
    bOnRamp = false;
    dKp = 0.4 / 24.0; // @max roll is 24, and max speed we want to go is 0.4
    dYaw = s_Swerve.getYawDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(s_Swerve.getRoll()) > 18.0) {
      bOnRamp = true;
      // dTrans = dTrans_remember * 0.8;
    }
    // if (bOnRamp && Math.abs(s_Swerve.getRoll()) < 9.0) {
    //   dTrans = 0.0;
    // }
    // else if(bOnRamp && Math.abs(s_Swerve.getRoll()) < 12.0) {
    //   dTrans = dTrans_remember * 0.5;
    // }
    if(bOnRamp) {
      dTrans = s_Swerve.getRoll() * dKp * 0.6; // when possible, change to .6
      if(Math.abs(dTrans) < 0.03) dTrans = 0.0;
    }
    
    if(Math.abs(s_Swerve.getYawDouble() - dYaw) > 15.0) dTrans = 0.0;
    
    /* Drive */
    s_Swerve.drive(
        new Translation2d(dTrans, dStrafe).times(Constants.Swerve.maxSpeed), 
        dRotate * Constants.Swerve.maxAngularVelocity, 
        !bRobotCentric, 
        true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
