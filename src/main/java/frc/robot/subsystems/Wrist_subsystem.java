// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.ctre.phoenix.Util;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import java.lang.Math;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities;
import edu.wpi.first.wpilibj.DigitalInput;


public class Wrist_subsystem extends SubsystemBase {

  private CANSparkMax objWristMotor = new CANSparkMax(Constants.canIDs.iWristMotor,MotorType.kBrushless);
  private DutyCycleEncoder objAbsEncoder;
  private double dSpeed;
  private boolean bSoftStopActive;
  private boolean bHoldPosition;
  private double dHoldAngle;
  // private DigitalInput objNegativeSide;    // For offset reset with limit switches
  // private DigitalInput objPositiveSide;    // For offset reset with limit switches
  // private double dOffset;
  // private double dOffsetLive;
  // private boolean bNegOld;
  // private boolean bPosOld;
  // private boolean bNeg;
  // private boolean bPos;
  private double dAngle;
  // private double dAngleNegSwitch = -131.9;
  // private double dAnglePosSwitch = 125.8;
  // private boolean bInitLimitSwitch;


  /** Creates a new Wrist_subsystem. */
  public Wrist_subsystem() {
    objWristMotor.setIdleMode(IdleMode.kBrake);
    objWristMotor.setSmartCurrentLimit(Constants.Wrist.iCurrentLimit);
    objAbsEncoder = new DutyCycleEncoder(Constants.Wrist.iDIOPort);
    // objNegativeSide = new DigitalInput(3);
    // objPositiveSide = new DigitalInput(4);
    // dOffsetLive = 0.0;

    // objAbsEncoder.setDistancePerRotation(Constants.Wrist.dDegreesPerRev);
    SmartDashboard.putNumber("Test Encoder", 0.0);
    SmartDashboard.putNumber("Test Offset", 0.0);
    SmartDashboard.putNumber("Test DegRev", 0.0);

    // bInitLimitSwitch = false;
    // init in case start with one or the other limit switches engaged
    // if (!objNegativeSide.get()) {
    //   if (Math.abs(getWristAngle() - dAngleNegSwitch) > 5.0) {
    //     dOffsetLive = -dAngleNegSwitch - Constants.Wrist.dOffset - (objAbsEncoder.get() * Constants.Wrist.dDegreesPerRev);
    //     bInitLimitSwitch = true;
    //   }
    // }

    // if (!objPositiveSide.get()) {
    // always do this at initialization, which requires wrist to be at or VERY near the limit switch when the robot powers on

      // if (Math.abs(getWristAngle() - dAnglePosSwitch) > 5.0) {
      //   dOffsetLive = -dAnglePosSwitch - Constants.Wrist.dOffset - (objAbsEncoder.get() * Constants.Wrist.dDegreesPerRev);
      //   bInitLimitSwitch = true;
      // }
    // }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // getWristAngle();
    dAngle = getWristAngle();
    // bPos = !objPositiveSide.get();
    // bNeg = !objNegativeSide.get();
    // double testResult = Utilities.correctAngle(SmartDashboard.getNumber("Test Encoder", 0.0), SmartDashboard.getNumber("Test Offset", 0.0), SmartDashboard.getNumber("Test DegRev", 0.0));
    // SmartDashboard.putNumber("Test Result", testResult);
    
    if (bSoftStopActive) {
      softStop();
      if (Math.abs(objWristMotor.get()) < 0.03) {
        bSoftStopActive = false;
        bHoldPosition = true;
        dHoldAngle = dAngle;
      }
    }

    // if (bHoldPosition) {
    //   holdPosition(dHoldAngle, 1.0);
    // } 

    // if start on a limit switch, constructor above corrects angle so don't do it the first time the limit switch turns off
    // if (bInitLimitSwitch) {
    //   bInitLimitSwitch = false;
    // }
    // else {    // every other time the limit switch turns off, recalibrate the angle
    //   // if (!bNeg && bNegOld) {
    //   //   if (Math.abs(dAngle - dAngleNegSwitch) > 5.0) {
    //   //     dOffsetLive = 0.0;
    //   //     dOffsetLive = dAngleNegSwitch - getWristAngle();
    //   //     dHoldAngle = getWristAngle();
    //   //   }
    //   // }
    //   // bNegOld = bNeg;
    //   if (bNeg) {
    //     if (Math.abs(dAngle - dAngleNegSwitch) > 5.0) {
    //       dOffsetLive = 0.0;
    //       dOffsetLive = -dAngleNegSwitch - Constants.Wrist.dOffset - (objAbsEncoder.get() * Constants.Wrist.dDegreesPerRev);
    //       dHoldAngle = getWristAngle();
    //     }
    //   }
    //   // bNegOld = bNeg;

    //   // if (!bPos && bPosOld) {
    //   //   if (Math.abs(dAngle - dAnglePosSwitch) > 5.0) {
    //   //     dOffsetLive = 0.0;
    //   //     dOffsetLive = getWristAngle() - dAnglePosSwitch;
    //   //     dHoldAngle = getWristAngle();
    //   //   }
    //   // }
    //   if (bPos) {
    //     if (Math.abs(dAngle - dAnglePosSwitch) > 5.0) {
    //       // dOffsetLive = 0.0;
    //       dOffsetLive = -dAnglePosSwitch - Constants.Wrist.dOffset - (objAbsEncoder.get() * Constants.Wrist.dDegreesPerRev);
    //       dHoldAngle = getWristAngle();
    //     }
    //   }
    //   // bPosOld = bPos;
    // }

    // SmartDashboard.putBoolean( "Positive", objPositiveSide.get());
    // SmartDashboard.putBoolean( "Negative", objNegativeSide.get());
    // SmartDashboard.putNumber("live Offset", dOffsetLive);
  }

  public void setSoftStop(boolean input) { 
    bSoftStopActive = input; 
    bHoldPosition = false;
  }
  
  public void moveWrist (double dSpeed) {
    double dSpeedLimit = Constants.Wrist.dSpeedControlMax;
    double dCurrentAngle = getWristAngle();
    // if (dCurrentAngle > Constants.Wrist.dMaxAngleLimit || !objPositiveSide.get()) {
    if (dCurrentAngle > Constants.Wrist.dMaxAngleLimit) {
    // if (!objPositiveSide.get()) {
      dSpeed = Utilities.limitVariable(0.0, dSpeed, dSpeedLimit);
    }
    // else if (dCurrentAngle < Constants.Wrist.dMinAngleLimit || !objNegativeSide.get()) {
    else if (dCurrentAngle < Constants.Wrist.dMinAngleLimit) {
    // else if (!objNegativeSide.get()) {
      dSpeed = Utilities.limitVariable(-dSpeedLimit, dSpeed, 0.0);
    }
    objWristMotor.set(dSpeed);
  }

  public void stopWrist() {
    objWristMotor.stopMotor();
  }

  public double softStop() {
    dSpeed = objWristMotor.get();
    if (dSpeed > 0.0) {
      dSpeed = Math.max(dSpeed - Constants.Arm.dSoftStopLimit, 0.0);
    }
    else {
      dSpeed = Math.min(dSpeed + Constants.Arm.dSoftStopLimit, 0.0);
    }
    if(Math.abs(dSpeed) < 0.035) {
      dSpeed = 0.0;
    }
    objWristMotor.set(dSpeed);
    return Math.abs(dSpeed);
  }

  public double getWristAngle() {
    double dWristAngle;
    // apply - here if + motor speed results in a decreasing angle (make it dWristAngle = -Utilities.correct...)
    // dWristAngle = -Utilities.correctAngle(objAbsEncoder.get(), Constants.Wrist.dOffset, Constants.Wrist.dDegreesPerRev);
    // dOffset = Constants.Wrist.dOffset + dOffsetLive;

    dWristAngle = Utilities.correctAngle2(objAbsEncoder.get(), Constants.Wrist.dOffset, 1.0, true);

    SmartDashboard.putNumber("Raw Wrist Encoder", objAbsEncoder.get());
    SmartDashboard.putNumber("Wrist Angle", dWristAngle);
    
    return dWristAngle;
  }

  public double moveWristToAngle(double dTargetAngle, double dAngle_old, double dCommand_old, double dSpeedMult) {
    double dSpeedLimit = Constants.Wrist.dSpeedControlMax;
    double dCurrentAngle = getWristAngle();
    double dDifference = dTargetAngle - dCurrentAngle; 
    double dDeriv;
    boolean bArrived = false;

    // computes dCommand, the motor speed
    dDeriv = dCurrentAngle - dAngle_old;
    double dCommand = -(dDifference * Constants.Wrist.kP - dDeriv * Constants.Wrist.kD);
    // if(Math.abs(dDifference) < 0.75) dCommand = 0.0;

    dCommand = Utilities.limitVariable(-dSpeedLimit * dSpeedMult, dCommand, dSpeedLimit * dSpeedMult);
    if (Math.abs(dCommand) > Math.abs(dCommand_old)) {      //Checking that speed is increasing
      dCommand = dCommand_old + Math.min(Math.abs(dCommand - dCommand_old), Constants.Wrist.dSpeedUpLimit) * Math.signum(dCommand);
    }
    moveWrist(dCommand);
    if (Math.abs(dDifference) < Constants.Wrist.dTolerance) {
      bArrived = true;
    }
    SmartDashboard.putBoolean("Wrist Arrived", bArrived);
    SmartDashboard.putNumber("WristControlSpeed", dCommand);
    return dCommand;
  }

  public void holdPosition(double dTargetAngle, double dSpeedMult) {
    double dSpeedLimit = Constants.Wrist.dSpeedControlMax;
    double dCurrentAngle = getWristAngle();
    double dDifference = dTargetAngle - dCurrentAngle; 
    double dCommand = -dDifference * (Constants.Wrist.kP * 0.6);
    dCommand = Utilities.limitVariable(-dSpeedLimit * dSpeedMult, dCommand, dSpeedLimit * dSpeedMult);
    moveWrist(dCommand);
  }

  // public void resetOffsetNegative() {
  //   dOffsetLive = 0.0;
  // }

  // public void resetOffsetPositive() {
  //   dOffsetLive = 0.0;
  // }
}