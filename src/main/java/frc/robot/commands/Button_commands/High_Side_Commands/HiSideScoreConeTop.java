// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Button_commands.High_Side_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist_subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Arm_subsystem;
import frc.robot.subsystems.Forearm_subsystem;

public class HiSideScoreConeTop extends CommandBase {

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
  double dArmTarget = 26.7;
  double dForearmTarget = 10.0;
  double dWristTarget = 65.0;

  /** Creates a new ScoreCubeTop. */
  public HiSideScoreConeTop(Arm_subsystem objArm_in, Forearm_subsystem objForearm_in, Wrist_subsystem objWrist_in) {
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
    iState = 1;
    objArm.resetRamp();
    objForearm.resetRamp();
    objWrist.resetRamp();

    bDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (iState) {
      case 1:         
        objArm.stopHoldingAngle();
        objForearm.stopHoldingAngle();
        objWrist.stopHoldingAngle();
        iState = 14;
        break;
      case 14:          // Move everything to "Pickup Verticle Cone" targets
        dArmCommand_old = objArm.moveArmToAngle2(dArmTarget, dArmCommand_old);
        dForearmCommand_old = objForearm.moveForearmToAngle2(dForearmTarget, dForearmCommand_old);
        dWristCommand_old = objWrist.moveWristToAngle2(dWristTarget, dWristCommand_old);
        // if all three joints are at correct angle then iState = 99;
        if (Math.abs(objForearm.getForearmAngle() - dForearmTarget) < Constants.Forearm.dTolerance && Math.abs(objArm.getArmAngle() - dArmTarget) < Constants.Arm.dTolerance && Math.abs(objWrist.getWristAngle() - dWristTarget) < Constants.Wrist.dTolerance) iState = 99;
        break;
      case 99:
        objArm.holdPosition(dArmTarget);
        objForearm.holdPosition(dForearmTarget);
        objWrist.holdPosition(dWristTarget);
        break;
    }
    // System.out.println("ScoreConeTop - state: " + iState);     //For Testing
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
