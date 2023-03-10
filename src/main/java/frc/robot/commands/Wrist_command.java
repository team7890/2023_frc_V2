// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist_subsystem;

public class Wrist_command extends CommandBase {

  private final double dSpeed;
  private final Wrist_subsystem objWrist_subsystem;
  private final boolean bMode;
  private final double dTargetAngle;
  private double dAngle_old;
  private double dCommand_old;
  private boolean bDone;

  /** Creates a new Wrist_command. */
  public Wrist_command(Wrist_subsystem objWrist_subsystem_in, double dSpeed_in, boolean bMode_in, double dTargetAngle_in) {
    objWrist_subsystem = objWrist_subsystem_in;
    dSpeed = dSpeed_in;
    bMode = bMode_in;       //if true moving Arm to an angle, if false move manually a fixed speed
    dTargetAngle = dTargetAngle_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objWrist_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    objWrist_subsystem.setSoftStop(false);
    dAngle_old = objWrist_subsystem.getWristAngle();
    dCommand_old = 0.0;
    bDone = false;
    System.out.println("Wrist_command init");  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // objWrist_subsystem.moveWrist(dSpeed);
    if (bMode) {
      dCommand_old = objWrist_subsystem.moveWristToAngle(dTargetAngle, dAngle_old, dCommand_old, 1.0);
      dAngle_old = objWrist_subsystem.getWristAngle();
      if (Math.abs(dTargetAngle - dAngle_old) < Constants.Wrist.dTolerance) {
        bDone = true; 
      }
    }
    else {
      objWrist_subsystem.moveWrist(dSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objWrist_subsystem.setSoftStop(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bDone;
  }
}
