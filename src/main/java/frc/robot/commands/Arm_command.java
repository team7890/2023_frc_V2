// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm_subsystem;

import java.lang.Math;

public class Arm_command extends CommandBase {

  private final double dSpeed;
  private final Arm_subsystem objArm_subsystem;
  private final boolean bMode;
  private final double dTargetAngle;
  private double dAngle_old;
  private double dCommand_old;
  private boolean bDone;

  /** Creates a new Arm_command. */
  public Arm_command(Arm_subsystem objArm_subsystem_in, double dSpeed_in, boolean bMode_in, double dTargetAngle_in) {
    objArm_subsystem = objArm_subsystem_in;
    dSpeed = dSpeed_in;
    bMode = bMode_in;       //if true moving Arm to an angle, if false move manually a fixed speed
    dTargetAngle = dTargetAngle_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objArm_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    objArm_subsystem.stopArm();
    dAngle_old = objArm_subsystem.getArmAngle();
    dCommand_old = 0.0;
    bDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // objArm_subsystem.moveArm(dSpeed);
    if (bMode) {
      dCommand_old = objArm_subsystem.moveArmToAngle(dTargetAngle, dAngle_old, dCommand_old, 1.0);
      dAngle_old = objArm_subsystem.getArmAngle();
      if (Math.abs(dTargetAngle - dAngle_old) < Constants.Arm.dTolerance) {
        bDone = true;
      }
    }
    else {
      dCommand_old = objArm_subsystem.moveArm(dSpeed, dCommand_old);
    }   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objArm_subsystem.setSoftStop(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bDone;
  }
}
