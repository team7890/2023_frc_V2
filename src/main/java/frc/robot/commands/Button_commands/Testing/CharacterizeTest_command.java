// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Button_commands.Testing;

import frc.robot.subsystems.Forearm_subsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CharacterizeTest_command extends CommandBase {

  private final Forearm_subsystem objForearm_subsystem;
  private double dSpeed;
  private double dMaxSpeed = 0.45;
  
  /** Creates a new CharacterizeTest_command. */
  public CharacterizeTest_command(Forearm_subsystem objForearm_in) {
    // Use addRequirements() here to declare subsystem dependencies.
    objForearm_subsystem = objForearm_in;
    addRequirements(objForearm_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dSpeed = 0.0;
    objForearm_subsystem.resetRamp();
    objForearm_subsystem.stopHoldingAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dSpeed = objForearm_subsystem.moveForearmToAngle2(90.0, dSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // objForearm_subsystem.setSoftStop(true);
    objForearm_subsystem.setHoldAngle(90.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dSpeed == 0.0;
  }
}
