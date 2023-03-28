// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import java.lang.Math;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities;


public class Wrist_subsystem extends SubsystemBase {

  private CANSparkMax objWristMotor = new CANSparkMax(Constants.canIDs.iWristMotor,MotorType.kBrushless);
  private DutyCycleEncoder objAbsEncoder;
  private double dSpeed;
  private boolean bSoftStopActive;
  private boolean bSoftStopToHold;
  private boolean bHoldPosition;
  private double dHoldAngle = -99.0;
  private double dAngle;
  private double dCharSpeed;
  private boolean bRampStop;
  private boolean bArrived;

  /** Creates a new Wrist_subsystem. */
  public Wrist_subsystem() {
    objWristMotor.setIdleMode(IdleMode.kBrake);
    objWristMotor.setSmartCurrentLimit(Constants.Wrist.iCurrentLimit);
    objAbsEncoder = new DutyCycleEncoder(Constants.Wrist.iDIOPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dAngle = getWristAngle();
    if (bSoftStopActive) {
      softStop();
      if (Math.abs(objWristMotor.get()) < 0.08) {
        bSoftStopActive = false;
        bHoldPosition = true;
        if (!bSoftStopToHold) dHoldAngle = dAngle;
      }
    }
    if (bHoldPosition) {
      holdPosition(dHoldAngle);
    }
    if (Math.abs(dHoldAngle - dAngle) < Constants.Wrist.dTolerance) bArrived = true;
    else bArrived = false;
    SmartDashboard.putBoolean("Wrist Arrived", bArrived);
    SmartDashboard.putNumber("Wrist Hold Angle", dHoldAngle);

  }

  public void setHoldAngle(double dHoldAngle_in) {
    dHoldAngle = dHoldAngle_in;
    bHoldPosition = true;
  }
  
  public void stopHoldingAngle() {
    bHoldPosition = false;
  }

  public void setSoftStop(boolean input) { 
    bSoftStopActive = input; 
    bSoftStopToHold = false;
    bHoldPosition = false;
  }

  public void setSoftStopToHold(double dHoldAngle_in) {
    bSoftStopActive = true;
    bSoftStopToHold = true;
    bHoldPosition = false;
    dHoldAngle = dHoldAngle_in;
  }
  
  public void moveWrist(double dSpeed) {
    double dSpeedLimit = Constants.Wrist.dSpeedControlMax;
    double dCurrentAngle = getWristAngle();
    // if (dCurrentAngle > Constants.Wrist.dMaxAngleLimit || !objPositiveSide.get()) {
    if (dCurrentAngle > Constants.Wrist.dMaxAngleLimit) {
    // if (!objPositiveSide.get()) {
      dSpeed = Utilities.limitVariable(-dSpeedLimit, dSpeed, 0.0);
    }
    // else if (dCurrentAngle < Constants.Wrist.dMinAngleLimit || !objNegativeSide.get()) {
    else if (dCurrentAngle < Constants.Wrist.dMinAngleLimit) {
    // else if (!objNegativeSide.get()) {
      dSpeed = Utilities.limitVariable(0.0, dSpeed, dSpeedLimit);
    }
    objWristMotor.set(dSpeed);
  }

  public void moveWrist2(double dSpeed) {
    double dSpeedLimit = Constants.Wrist.dSpeedControlMax;
    double dCurrentAngle = getWristAngle();
    if (dCurrentAngle > Constants.Wrist.dMaxAngleLimit) {
      dSpeed = Utilities.limitVariable(-dSpeedLimit, dSpeed, 0.0);   
    }
    else if (dCurrentAngle < Constants.Wrist.dMinAngleLimit) {
      dSpeed = Utilities.limitVariable(0.0, dSpeed, dSpeedLimit); 
    }
    objWristMotor.set(dSpeed);
  }

  public double characterize(double dSpeed_in, double dMaxSpeed_in) {
    if (!bRampStop) {
      dCharSpeed = dSpeed_in + Constants.Forearm.dRampLimit;
    }
    else {
      dCharSpeed = Math.max(dSpeed_in - Constants.Forearm.dRampLimit, 0.0);
    }

    if (dCharSpeed >= dMaxSpeed_in) {
      bRampStop = true;
    }

    moveWrist2(dCharSpeed);
    return dCharSpeed;
  }
  
  public void resetRamp() {
    bRampStop = false;
  }

  public void stopWrist() {
    objWristMotor.stopMotor();
  }

  public double softStop() {
    dSpeed = objWristMotor.get();
    if (dSpeed > 0.0) {
      dSpeed = Math.max(dSpeed - Constants.Wrist.dSoftStopLimit, 0.0);
    }
    else {
      dSpeed = Math.min(dSpeed + Constants.Wrist.dSoftStopLimit, 0.0);
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

    dWristAngle = Utilities.correctAngle2(objAbsEncoder.get(), Constants.Wrist.dOffset, 1.0, false);

    SmartDashboard.putNumber("Raw Wrist Encoder", objAbsEncoder.get());
    SmartDashboard.putNumber("Wrist Angle", dWristAngle);
    
    return dWristAngle;
  }

  public double moveWristToAngle(double dTargetAngle, double dAngle_old, double dCommand_old, double dSpeedMult) {
    double dSpeedLimit = Constants.Wrist.dSpeedControlMax;
    double dCurrentAngle = getWristAngle();
    double dDifference = dTargetAngle - dCurrentAngle; 
    double dDeriv;
    double dCommand;

    // computes dCommand, the motor speed
    dDeriv = dCurrentAngle - dAngle_old;
    if (Math.abs(dDifference) > 2.0) {
      dCommand = dDifference * Constants.Wrist.kP - dDeriv * Constants.Wrist.kD;
    }
    else {
      dCommand = dDifference * Constants.Wrist.kP;
    }    // if(Math.abs(dDifference) < 0.75) dCommand = 0.0;

    dCommand = Utilities.limitVariable(-dSpeedLimit * dSpeedMult, dCommand, dSpeedLimit * dSpeedMult);
    if (Math.abs(dCommand) > Math.abs(dCommand_old)) {      //Checking that speed is increasing
      dCommand = dCommand_old + Math.min(Math.abs(dCommand - dCommand_old), Constants.Wrist.dSpeedUpLimit) * Math.signum(dCommand);
    }
    moveWrist(dCommand);
    return dCommand;
  }

  public double moveWristToAngle2(double dTargetAngle, double dCommand_old) {
    // note positive arm speed moves
    double dSpeedLimit = Constants.Wrist.dSpeedControlMax;
    double dCurrentAngle = getWristAngle();
    double dDifference = dTargetAngle - dCurrentAngle; // To see if we have arrived
    double dCommand = 0.0;
    double dSign = Math.signum(dDifference);
    double dRampDownAngle;

    if (!bRampStop) {
      // if we haven't reached max speed, speed up to speed limit which is cruising speed
      dCommand = dCommand_old + dSign * Constants.Wrist.dRampLimit;
    }
    dCommand = Utilities.limitVariable(-dSpeedLimit, dCommand, dSpeedLimit);
    dRampDownAngle = 0.5 * Constants.Wrist.dRampCharKvalue * dCommand_old * dCommand_old / Constants.Wrist.dRampLimit;
    if (Math.abs(dDifference) <= dRampDownAngle) {
      // if we are to the point where we need to slow down to arrive at the angle, set bRampStop to true to do the ramp down
      bRampStop = true;
    }
    // else bRampStop = false;

    if (bRampStop) {
      dCommand = dCommand_old - dSign * Constants.Wrist.dRampLimit;
      if (dSign > 0.0) {
        // limit speed so do not ramp past zero when speed is positive
        dCommand = Math.max(dCommand, 0.0);
      }
      else {
        // limit speed so do not ramp past zero when speed is negative
        dCommand = Math.min(dCommand, 0.0);
      }
    }

    moveWrist2(dCommand);
    return dCommand;
  }


  public void holdPosition(double dTargetAngle) {
    double dSpeedLimit = Constants.Wrist.dSpeedControlMax;
    double dCurrentAngle = getWristAngle();
    double dDifference = dTargetAngle - dCurrentAngle;
    double dCommand = dDifference * 0.011;
    dCommand = Utilities.limitVariable(-dSpeedLimit, dCommand, dSpeedLimit);
    moveWrist2(dCommand);
  }

}