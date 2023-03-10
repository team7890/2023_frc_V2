// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Button_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm_subsystem;
import frc.robot.subsystems.Forearm_subsystem;
import frc.robot.subsystems.Wrist_subsystem;

public class ScoreConeMiddle2 extends CommandBase {

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
  double dArmTarget = -22.0;
  double dForearmTarget = -50.2;
  double dWristTarget = -3.6;

  /** Creates a new ScoreCubeTop. */
  public ScoreConeMiddle2(Arm_subsystem objArm_in, Forearm_subsystem objForearm_in, Wrist_subsystem objWrist_in) {
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
    if (objForearm.getForearmAngle() > -20.0) iState = 10;
    else if (objForearm.getForearmAngle() < -100.0) iState = 20;
    else iState = 21;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (iState) {

      case 10:    //if the forearm is out on the high scoring side first move the wirst up
        dWristCommand_old = objWrist.moveWristToAngle(-45.0, dWristAngle_old, dWristCommand_old, 3.0);
        if (objWrist.getWristAngle() < -10.0) iState = 11;
        break;
      case 11:    // once Arm in within robot start Arm and Forearm to target keep wrist goint to -45.0 angle
        dArmCommand_old = objArm.moveArmToAngle(dArmTarget, dArmAngle_old, dArmCommand_old, 5.0);
        dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 3.0);
        dWristCommand_old = objWrist.moveWristToAngle(-45.0, dWristAngle_old, dWristCommand_old, 3.0);
        if (objForearm.getForearmAngle() < -20.0) iState = 21;    // Once forarm is past verticle swap to case 
        break;
      // case 13:    // Move everything to "Score Cone Middle" targets
      //   dArmCommand_old = objArm.moveArmToAngle(dArmTarget, dArmAngle_old, dArmCommand_old, 2.0);
      //   dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 0.5);
      //   dWristCommand_old = objWrist.moveWristToAngle(-100.0, dWristAngle_old, dWristCommand_old, 5.0);
      //   // if Arm and Forearm joints are at correct angle then iState = 14;
      //   if (objForearm.getForearmAngle() < -30.0) iState = 21;
      //   break;
      case 20: 
        objArm.softStop();
        dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 4.0);
        objWrist.softStop();
        if (objForearm.getForearmAngle() >= -100.0) iState = 21;
      case 21:
        objArm.softStop();
        objForearm.softStop();
        dWristCommand_old = objWrist.moveWristToAngle(-100.0, dWristAngle_old, dWristCommand_old, 4.0);
        if (objWrist.getWristAngle() < -95.0) iState = 22;
        break;
      case 22:
        dArmCommand_old = objArm.moveArmToAngle(dArmTarget, dArmAngle_old, dArmCommand_old, 5.0);
        dForearmCommand_old = objForearm.moveForearmToAngle(dForearmTarget, dForearmAngle_old, dForearmCommand_old, 4.0);
        dWristCommand_old = objWrist.moveWristToAngle(45.0, dWristAngle_old, dWristCommand_old, 5.0);
        if (Math.abs(objForearm.getForearmAngle() - dForearmTarget) < 1.0 && Math.abs(objArm.getArmAngle() - dArmTarget) < 1.0 && objWrist.getWristAngle() > 40.0) iState = 33;
        break;
      case 33:
        objArm.softStop();
        objForearm.softStop();
        dWristCommand_old = objWrist.moveWristToAngle(dWristTarget, dWristAngle_old, dWristCommand_old, 3.0);
        if (Math.abs(objForearm.getForearmAngle() - dForearmTarget) < 1.0 && Math.abs(objArm.getArmAngle() - dArmTarget) < 1.0 && Math.abs(objWrist.getWristAngle() - dWristTarget) < 1.0) iState = 99;
        break;
      case 99:
        objArm.softStop();
        objForearm.softStop();
        objWrist.softStop();
        break;
    }
    dArmAngle_old = objArm.getArmAngle();
    dForearmAngle_old = objForearm.getForearmAngle();
    dWristAngle_old = objWrist.getWristAngle();
    System.out.println("ScoreConeMiddle2 - state: " + iState);     //For Testing
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