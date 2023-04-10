// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Roller_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.RollerHand_subsystem;

public class ConeIntake_ForSingleSub_command extends CommandBase {

  private final RollerHand_subsystem objRollerHand;
  private double dMaxCurrent;
  private double dStartTime;
  private boolean bInrushCurrentPeriodDone;
  private boolean bCurrentLimitTripped;


  /** Creates a new ConeIntake. */
  public ConeIntake_ForSingleSub_command(RollerHand_subsystem objRollerHand_in) {
    objRollerHand = objRollerHand_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objRollerHand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dMaxCurrent = 0.0;
    objRollerHand.stopMotors();
    dStartTime = Timer.getFPGATimestamp();
    bInrushCurrentPeriodDone = false;
    bCurrentLimitTripped = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - dStartTime > 0.35) {
      bInrushCurrentPeriodDone = true;
    }
    else {
      dMaxCurrent = 0.0;
    }
    if (bInrushCurrentPeriodDone && objRollerHand.getMotor1Current() > 15.0) {
      bCurrentLimitTripped = true;
    }
    if (bCurrentLimitTripped) {
      objRollerHand.stopMotors();
    }
    else {
      objRollerHand.intakeCone_SingleSubstation();
    }
    dMaxCurrent = Math.max(dMaxCurrent, objRollerHand.getMotor1Current());
    // SmartDashboard.putNumber("Max Roller Current", dMaxCurrent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objRollerHand.holdCone();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
