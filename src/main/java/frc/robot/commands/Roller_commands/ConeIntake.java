// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Roller_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.RollerHand_subsystem;

public class ConeIntake extends CommandBase {

  private final RollerHand_subsystem objRollerHand;
  private int iCount;

  /** Creates a new ConeIntake. */
  public ConeIntake(RollerHand_subsystem objRollerHand_in) {
    objRollerHand = objRollerHand_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objRollerHand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    iCount = 0;
    objRollerHand.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (objRollerHand.getMotor1Current() > 4.0) {
      iCount = iCount + 1;
    }
    else {
      iCount = 0;
      objRollerHand.intakeCone();
    }
    if (iCount > 8) {
      objRollerHand.stopMotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objRollerHand.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
