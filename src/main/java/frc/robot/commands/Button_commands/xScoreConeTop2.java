// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Button_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist_subsystem;
import frc.robot.subsystems.Arm_subsystem;
import frc.robot.subsystems.Forearm_subsystem;

public class xScoreConeTop2 extends CommandBase {

  private final Wrist_subsystem objWrist;
  private final Forearm_subsystem objForearm;
  private final Arm_subsystem objArm;
  //Arm Variables
  private double dArmAngle_old;
  private double dArmCommand_old;
  //Forearm Variables
  private double dForearmAngle_old;
  private double dForearmCommand_old;
  //Wrist Variables
  private double dWristAngle_old;
  private double dWristCommand_old;

  private int iState;
  private int iCounter;


  // Final Target Positions
  double dArmTarget = 25.0;
  double dForearmTarget = 29.5;
  double dWristTarget = 0.0;

  /** Creates a new ScoreConeTop. */
  public xScoreConeTop2(Arm_subsystem objArm_in, Forearm_subsystem objForearm_in, Wrist_subsystem objWrist_in) {
    objArm = objArm_in;
    objForearm = objForearm_in;
    objWrist = objWrist_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objArm);
    addRequirements(objForearm);
    addRequirements(objWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Arm Variables
    objArm.setSoftStop(false);
    dArmAngle_old = objArm.getArmAngle();
    dArmCommand_old = 0.0;
    //Forearm Variables
    objForearm.setSoftStop(false);
    dForearmAngle_old = objForearm.getForearmAngle();
    dForearmCommand_old = 0.0;
    //Wrist Variables
    objWrist.setSoftStop(false);
    dWristAngle_old = objWrist.getWristAngle();
    dWristCommand_old = 0.0;

    iState = 20;                                            // Forearm is already on the high scoring side of the robot.
    if (objForearm.getForearmAngle() < 15.0) iState = 10;   // Forearm is on the non-High scoring side of the robot.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (iState) {
      case 10:  // forearm is on the non-high-scoring side of the robot and...
                // moving arm to a position just inside the robot to make sure we don't go outside on both sides
                // moving forearm to the target which is up and over the top
                // moving wrist to the closer of -35.0 and 35.0 to go over the top and not reach too high
        dArmCommand_old = objArm.moveArmToAngle(-5.0, dArmAngle_old, dArmCommand_old, 2.0);
        dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 3.0);
        if (objWrist.getWristAngle() > 0.0) {
          dWristCommand_old = objWrist.moveWristToAngle(35.0, dWristAngle_old, dWristCommand_old, 3.0);
        }
        else {
          dWristCommand_old = objWrist.moveWristToAngle(-35.0, dWristAngle_old, dWristCommand_old, 3.0);
        }
        if (objForearm.getForearmAngle() > 20.0) iState = 11;
        iCounter = 0;
        break;
      case 11:  // forearm has gotten to 20 degrees on the high scoring side and we will pause in this position for 300 ms or so
                // in order to flip the cone
        objArm.softStop();
        objForearm.softStop();
        objWrist.softStop();
        iCounter = iCounter + 1;
        if (iCounter > 15) iState = 20; // cone has flipped, now proceed to the three target angles
        break;
      case 20:  // proceed to the three target angles
        dArmCommand_old = objArm.moveArmToAngle(dArmTarget, dArmAngle_old, dArmCommand_old, 2.0);
        dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 2.0);
        dWristCommand_old = objWrist.moveWristToAngle(dWristTarget, dWristAngle_old, dWristCommand_old, 3.0);
        // if all wrist joint is at correct angle then iState = 99;
        if (Math.abs(objForearm.getForearmAngle() - dForearmTarget) < 1.0 && Math.abs(objArm.getArmAngle() - dArmTarget) < 1.0 && Math.abs(objWrist.getWristAngle() - dWristTarget) < 1.0) iState = 99;
        break;
      case 99:
        objArm.softStop();
        objForearm.softStop();
        objWrist.softStop();
        break;
    }
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
