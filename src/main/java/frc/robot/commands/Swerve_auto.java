// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve_subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Swerve_auto extends CommandBase {
  private Swerve_subsystem s_Swerve;
  private double dTrans;
  private double dStrafe;
  private double dRotate;
  private boolean bRobotCentric;

  /** Creates a new AutoSwerve. */
  public Swerve_auto(Swerve_subsystem s_Swerve_in, double dTrans_in, double dStrafe_in, double dRotate_in, boolean bRobotCentric_in) {
    s_Swerve = s_Swerve_in;
    dTrans = dTrans_in;
    dStrafe = dStrafe_in;
    dRotate = dRotate_in;
    bRobotCentric = bRobotCentric_in;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get Values, Deadband*/

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
        /* Stop */
        s_Swerve.drive(
          new Translation2d(0.0, 0.0).times(Constants.Swerve.maxSpeed), 
          0.0 * Constants.Swerve.maxAngularVelocity, 
          !bRobotCentric, 
          true
      );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
