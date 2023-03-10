// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Button_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist_subsystem;
import frc.robot.subsystems.Arm_subsystem;
import frc.robot.subsystems.Forearm_subsystem;

public class ScoreConeTop3 extends CommandBase {

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
  double dArmTarget = 25.0;
  double dForearmTarget = 28.3;
  // double dWristTarget = -34.3;
  double dWristTarget = -15.0;

  /** Creates a new ScoreCubeTop. */
  public ScoreConeTop3(Arm_subsystem objArm_in, Forearm_subsystem objForearm_in, Wrist_subsystem objWrist_in) {
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

    iState = 0;
    if (objForearm.getForearmAngle() < 20.0) iState = 10;
    else if (objForearm.getForearmAngle() >= 20.0) iState = 22;
    else iState = 13;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (objForearm.getForearmAngle() < 30.0) {
    //   dArmCommand_old = objArm.moveArmToAngle(-5.0, dArmAngle_old, dArmCommand_old, 2.0);
    //   dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 3.0);
    //   if (objWrist.getWristAngle() > 0.0) {
    //     dWristCommand_old = objWrist.moveWristToAngle(35.0, dWristAngle_old, dWristCommand_old, 1.0);
    //   }
    //   else {
    //     dWristCommand_old = objWrist.moveWristToAngle(-35.0, dWristAngle_old, dWristCommand_old, 1.0);
    //   }
    // }
    // else {
    //   dArmCommand_old = objArm.moveArmToAngle(dArmTarget, dArmAngle_old, dArmCommand_old, 2.0);
    //   dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 1.0);
    //   dWristCommand_old = objWrist.moveWristToAngle(dWristTarget, dWristAngle_old, dWristCommand_old, 2.0);
    // }
    // dArmAngle_old = objArm.getArmAngle();
    // dForearmAngle_old = objForearm.getForearmAngle();
    // dWristAngle_old = objWrist.getWristAngle();
    switch (iState) {
      case 10:          //if the forearm is out on the pickup side first move the wirst up
        dArmCommand_old = objArm.moveArmToAngle(0.0, dArmAngle_old, dArmCommand_old, 5.0);
        objForearm.softStop();
        dWristCommand_old = objWrist.moveWristToAngle(100.0, dWristAngle_old, dWristCommand_old, 2.0);
        if (objWrist.getWristAngle() > 10.0) iState = 11;
        break;
      case 11:          // Move Arm to 0.0 so it doesn't bounce out of frame when moving forearm over the top.
        dArmCommand_old = objArm.moveArmToAngle(0.0, dArmAngle_old, dArmCommand_old, 5.0);
        dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 3.0);
        dWristCommand_old = objWrist.moveWristToAngle(100.0, dWristAngle_old, dWristCommand_old, 2.0);
        if (objForearm.getForearmAngle() >= 20.0) iState = 13;
        break;
      case 13:          // once the forearm gets over the top (towards place top cone position) then move wrist to target 
                        // (with forearm going to target)
        dArmCommand_old = objArm.moveArmToAngle(dArmTarget, dArmAngle_old, dArmCommand_old, 2.0);
        objForearm.softStop();
        dWristCommand_old = objWrist.moveWristToAngle(100.0, dWristAngle_old, dWristCommand_old, 5.0);
        if (objWrist.getWristAngle() > 95.0) iState = 14;
        break;
      // case 22:
      //   dArmCommand_old = objArm.moveArmToAngle(dArmTarget, dArmAngle_old, dArmCommand_old, 2.0);
      //   dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 3.0);
      //   dWristCommand_old = objWrist.moveWristToAngle(-20.0, dWristAngle_old, dWristCommand_old, 2.0);
      //   if (Math.abs(objForearm.getForearmAngle() - dForearmTarget) < 1.0 && Math.abs(objArm.getArmAngle() - dArmTarget) < 1.0 && objWrist.getWristAngle() < -40.0) iState = 14;
      //   break;
      case 14:          // Move everything to their Top Cone targets
        dArmCommand_old = objArm.moveArmToAngle(dArmTarget, dArmAngle_old, dArmCommand_old, 5.0);
        dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 3.0);
        dWristCommand_old = objWrist.moveWristToAngle(dWristTarget, dWristAngle_old, dWristCommand_old, 2.0);
        // if all three joints are at correct angle then iState = 99;
        if (Math.abs(objForearm.getForearmAngle() - dForearmTarget) < 1.0 && Math.abs(objArm.getArmAngle() - dArmTarget) < 1.0 && Math.abs(objWrist.getWristAngle() - dWristTarget) < 1.0) iState = 99;
        break;
      case 99:
        objArm.setSoftStop(true);
        objForearm.setSoftStop(true);
        objWrist.setSoftStop(true);
        break;
    }
    dArmAngle_old = objArm.getArmAngle();
    dForearmAngle_old = objForearm.getForearmAngle();
    dWristAngle_old = objWrist.getWristAngle();
    // System.out.println("ScoreConeTop3 - state: " + iState);     //For Testing
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
