// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Button_commands.Regular_Side_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist_subsystem;
import frc.robot.subsystems.Arm_subsystem;
import frc.robot.subsystems.Forearm_subsystem;

public class RegStowArm extends CommandBase {

  private final Wrist_subsystem objWrist;
  private final Forearm_subsystem objForearm;
  private final Arm_subsystem objArm;
  //Arm Variables
  private double dArmCommand_old;
  //Forearm Variables
  private double dForearmCommand_old;
  //Wrist Variables
  private double dWristCommand_old;

  private int iState;
  private boolean bDone;

  // Final Target Positions
  double dArmTarget = 3.5;
  double dForearmTarget = -150.0;
  double dWristTarget = 114.5;


  /** Creates a new ScoreCubeTop. */
  public RegStowArm(Arm_subsystem objArm_in, Forearm_subsystem objForearm_in, Wrist_subsystem objWrist_in) {
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
    iState = 0;
    objArm.resetRamp();
    objForearm.resetRamp();
    objWrist.resetRamp();
    if (objForearm.getForearmAngle() > -20.0) iState = 12;
    else iState = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (iState) {
      case 10:         
        objArm.stopHoldingAngle();
        objWrist.stopHoldingAngle();
        iState = 11;
        break;
      case 11:          //if the forearm is out on the high scoring side
                        // first move the wirst up (means the wrist is going to a negative angle)
        dArmCommand_old = objArm.moveArmToAngle2(dArmTarget, dArmCommand_old);
        dWristCommand_old = objWrist.moveWristToAngle2(dWristTarget, dWristCommand_old);
        if (objWrist.getWristAngle() > 45.0 && objArm.getArmAngle() > -8.0) {
          objForearm.stopHoldingAngle();
          iState = 13;
        }
        break;
      case 12:         
        objArm.stopHoldingAngle();
        objForearm.stopHoldingAngle();
        objWrist.stopHoldingAngle();
        iState = 13;
        break;
      case 13:          // Move everything to stow targets
        dArmCommand_old = objArm.moveArmToAngle2(dArmTarget, dArmCommand_old);
        dForearmCommand_old = objForearm.moveForearmToAngle2(dForearmTarget, dForearmCommand_old);
        dWristCommand_old = objWrist.moveWristToAngle2(dWristTarget, dWristCommand_old);
        // if all wrist joint is at correct angle then iState = 99;
        if (Math.abs(objForearm.getForearmAngle() - dForearmTarget) < 1.0 && Math.abs(objArm.getArmAngle() - dArmTarget) < 1.0 && Math.abs(objWrist.getWristAngle() - dWristTarget) < 1.0) iState = 99;
        break;
      case 99:
        objArm.holdPosition(dArmTarget);
        objForearm.holdPosition(dForearmTarget);
        objWrist.holdPosition(dWristTarget);
        break;
    }
    // System.out.println("StowArm - state: " + iState);     //For Testing
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (iState == 99) {
      objArm.setSoftStopToHold(dArmTarget);
      objForearm.setSoftStopToHold(dForearmTarget);
      objWrist.setSoftStopToHold(dWristTarget);
    }
    else {
      objArm.setSoftStop(true);
      objForearm.setSoftStop(true);
      objWrist.setSoftStop(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}