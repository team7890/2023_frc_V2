// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Button_commands.Testing;

import frc.robot.subsystems.Arm_subsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class CharacterizeArm_command extends CommandBase {

  private final Arm_subsystem objArm_subsystem;
  private double dSpeed;
  private double dMaxSpeed = Constants.Arm.dSpeedControlMax;
  
  /** Creates a new CharacterizeTest_command. */
  public CharacterizeArm_command(Arm_subsystem objArm_in) {
    // Use addRequirements() here to declare subsystem dependencies.
    objArm_subsystem = objArm_in;
    addRequirements(objArm_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dSpeed = 0.0;
    objArm_subsystem.resetRamp();
    objArm_subsystem.stopHoldingAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // dSpeed = objArm_subsystem.characterize(dSpeed, dMaxSpeed);    // this line for characterize
    dSpeed = objArm_subsystem.moveArmToAngle2(15.0, dSpeed);   // this line to test
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // objArm_subsystem.setHoldAngle(objArm_subsystem.getArmAngle());    // this line for characterize
    objArm_subsystem.setHoldAngle(15.0);                           // this line to test
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dSpeed == 0.0;
  }
}
