// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Button_commands.Testing;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist_subsystem;
import frc.robot.Constants;

public class CharacterizeWrist_command extends CommandBase {
  private final Wrist_subsystem objWrist_subsystem;
  private double dSpeed;
  private double dMaxSpeed = Constants.Wrist.dSpeedControlMax;

  /** Creates a new CharacterizeWrist_command. */
  public CharacterizeWrist_command(Wrist_subsystem objWrist_in) {
    objWrist_subsystem = objWrist_in;    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objWrist_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dSpeed = 0.0;
    objWrist_subsystem.resetRamp();
    objWrist_subsystem.stopHoldingAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dSpeed = objWrist_subsystem.characterize(dSpeed, dMaxSpeed);      // this line for characterize
    // dSpeed = objWrist_subsystem.moveWristToAngle2(15.0, dSpeed);   // this line to test
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objWrist_subsystem.setHoldAngle(objWrist_subsystem.getWristAngle());    // this line for characterize
    // objWrist_subsystem.setHoldAngle(15.0);                               // this line to test
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dSpeed == 0.0;
  }
}
