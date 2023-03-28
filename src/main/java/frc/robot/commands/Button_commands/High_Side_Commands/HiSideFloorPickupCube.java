// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Button_commands.High_Side_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist_subsystem;
import frc.robot.subsystems.Arm_subsystem;
import frc.robot.subsystems.Forearm_subsystem;

public class HiSideFloorPickupCube extends CommandBase {

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


  // Final Target Positions
  double dArmTarget = 16.8;
  double dForearmTarget = 157.5;
  double dWristTarget = -111.5;


  /** Creates a new ScoreCubeTop. */
  public HiSideFloorPickupCube(Arm_subsystem objArm_in, Forearm_subsystem objForearm_in, Wrist_subsystem objWrist_in) {
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
    // //Arm Variables
    // // objArm.setSoftStop(false);
    // // objArm.stopHoldingAngle();
    // dArmAngle_old = objArm.getArmAngle();
    // dArmCommand_old = 0.0;
    // //Forearm Variables
    // // objForearm.setSoftStop(false);
    // // objForearm.stopHoldingAngle();
    // dForearmAngle_old = objForearm.getForearmAngle();
    // dForearmCommand_old = 0.0;
    // //Wrist Variables
    // // objWrist.setSoftStop(false);
    // // objWrist.stopHoldingAngle();
    // dWristAngle_old = objWrist.getWristAngle();
    // dWristCommand_old = 0.0;

    // iState = 0;
    // if (objForearm.getForearmAngle() > 120.0) iState = 10;
    // else iState = 14;
    iState = 14;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (iState) {
      // case 10:          //if the forearm is out on the high scoring side
      //                   // first move the wirst up (means the wrist is going to a negative angle)
      //   objArm.holdPosition(objArm.getArmAngle());
      //   dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 1.0);
      //   objWrist.holdPosition(objWrist.getWristAngle());
      //   if (objWrist.getWristAngle() > 45.0 && objArm.getArmAngle() > -8.0) iState = 14;
      //   break;
      case 11:

        break;
      case 14:          // Move everything to stow targets
        dArmCommand_old = objArm.moveArmToAngle(dArmTarget, dArmAngle_old, dArmCommand_old, 1.0);
        dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 1.0);
        dWristCommand_old = objWrist.moveWristToAngle(dWristTarget, dWristAngle_old, dWristCommand_old, 1.0);
        // if all wrist joint is at correct angle then iState = 99;
        if (Math.abs(objForearm.getForearmAngle() - dForearmTarget) < 1.0 && Math.abs(objArm.getArmAngle() - dArmTarget) < 1.0 && Math.abs(objWrist.getWristAngle() - dWristTarget) < 1.0) iState = 99;
        break;
      case 99:
        objArm.holdPosition(dArmTarget);
        objForearm.holdPosition(dForearmTarget);
        objWrist.holdPosition(dWristTarget);
        break;
    }
    dArmAngle_old = objArm.getArmAngle();
    dForearmAngle_old = objForearm.getForearmAngle();
    dWristAngle_old = objWrist.getWristAngle();
    // System.out.println("StowArm - state: " + iState);     //For Testing
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objArm.holdPosition(dArmTarget);
    objForearm.holdPosition(dForearmTarget);
    objWrist.holdPosition(dWristTarget);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}