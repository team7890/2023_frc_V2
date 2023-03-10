// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

//Our imports
import frc.robot.subsystems.xGrabber_subsystem;


public class xGrabber_command extends CommandBase {

  private final xGrabber_subsystem objGrabber_subsystem;
  private boolean bDone;

  /** Creates a new Grabber. */
  public xGrabber_command(xGrabber_subsystem objGrabber_subsystem_in) {
    
    objGrabber_subsystem = objGrabber_subsystem_in;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(objGrabber_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    objGrabber_subsystem.toggleGrabber();
    bDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bDone;
  }
}
