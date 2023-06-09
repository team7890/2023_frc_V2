// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Roller_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.RollerHand_subsystem;

public class CubeIntake_command extends CommandBase {

  private final RollerHand_subsystem objRollerHand;
  private double dSpeed_old;

  /** Creates a new ConeOuttake. */
  public CubeIntake_command(RollerHand_subsystem objRollerHand_in) {
    objRollerHand = objRollerHand_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objRollerHand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    objRollerHand.stopMotors();
    dSpeed_old = objRollerHand.getSpeed();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objRollerHand.intakeCube();
    // System.out.println("CubeIntake_command");     //For Testing
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objRollerHand.holdCube();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
