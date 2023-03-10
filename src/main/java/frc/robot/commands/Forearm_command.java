// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Forearm_subsystem;

import java.lang.Math;

public class Forearm_command extends CommandBase {

  private final double dSpeed;
  private final Forearm_subsystem objForearm_subsystem;
  private final boolean bMode;
  private final double dTargetAngle;
  private double dAngle_old;
  private double dCommand_old;
  private boolean bDone;

  /** Creates a new Forearm_command. */
  public Forearm_command(Forearm_subsystem objForearm_subsystem_in, double dSpeed_in, boolean bMode_in, double dTargetAngle_in) {
    objForearm_subsystem = objForearm_subsystem_in;
    dSpeed = dSpeed_in;
    bMode = bMode_in;       //if true moving Arm to an angle, if false move manually a fixed speed
    dTargetAngle = dTargetAngle_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objForearm_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    objForearm_subsystem.setSoftStop(false);
    // objForearm_subsystem.stopForearm();
    dAngle_old = objForearm_subsystem.getForearmAngle();
    dCommand_old = 0.0;
    bDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (bMode) {
      dCommand_old = objForearm_subsystem.moveForearmToAngle(dTargetAngle, dAngle_old, dCommand_old, 1.0);
      dAngle_old = objForearm_subsystem.getForearmAngle();
      if (Math.abs(dTargetAngle - dAngle_old) < Constants.Forearm.dTolerance) {
        bDone = true;
      }
    }
    else {
      objForearm_subsystem.moveForearm(dSpeed);
    }
    System.out.println("Forearm Target:  " + dTargetAngle);
    System.out.println("Forearm Angle:  " + dAngle_old);
    System.out.println("Forearm Done:  " + bDone);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objForearm_subsystem.setSoftStop(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bDone;
  }
}
