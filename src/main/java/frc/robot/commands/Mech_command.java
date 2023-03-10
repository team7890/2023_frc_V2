// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist_subsystem;
import frc.robot.subsystems.Arm_subsystem;
import frc.robot.subsystems.Forearm_subsystem;

public class Mech_command extends CommandBase {

  private final Wrist_subsystem objWrist;
  private final Forearm_subsystem objForearm;
  private final Arm_subsystem objArm;
  //Arm Variables
  private double dArmAngle_old;
  private double dArmCommand_old;
  private final double dArmTargetAngle;
  //Forearm Variables
  private double dForearmAngle_old;
  private double dForearmCommand_old;
  private final double dForearmTargetAngle;
  //Wrist Variables
  private double dWristAngle_old;
  private double dWristCommand_old;
  private final double dWristTargetAngle;
  

  /** Creates a new Mech_command. */
  public Mech_command(Arm_subsystem objArm_in, Forearm_subsystem objForearm_in, Wrist_subsystem objWrist_in, double dArmTargetAngle_in, double dForearmTargetAngle_in, double dWristTargetAngle_in) {
    objWrist = objWrist_in;
    objForearm = objForearm_in;
    objArm = objArm_in;
    dArmTargetAngle = dArmTargetAngle_in;
    dForearmTargetAngle = dForearmTargetAngle_in;
    dWristTargetAngle = dWristTargetAngle_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objWrist);
    addRequirements(objForearm);
    addRequirements(objArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Arm Variables
    dArmAngle_old = objArm.getArmAngle();
    dArmCommand_old = 0.0;
    //Forearm Variables
    dForearmAngle_old = objForearm.getForearmAngle();
    dForearmCommand_old = 0.0;
    //Wrist Variables
    dWristAngle_old = objWrist.getWristAngle();
    dWristCommand_old = 0.0;

    // Softstop Stuff
    objArm.setSoftStop(false);
    objForearm.setSoftStop(false);
    objWrist.setSoftStop(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dArmCommand_old = objArm.moveArmToAngle(dArmTargetAngle, dArmAngle_old, dArmCommand_old, 1.0);
    dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTargetAngle, dForearmAngle_old, dForearmCommand_old, 1.0);
    dWristCommand_old = objWrist.moveWristToAngle(dWristTargetAngle, dWristAngle_old, dWristCommand_old, 1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objArm.setSoftStop(true);
    objForearm.setSoftStop(true);
    objWrist.setSoftStop(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
