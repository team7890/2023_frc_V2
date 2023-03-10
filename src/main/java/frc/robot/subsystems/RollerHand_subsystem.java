// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class RollerHand_subsystem extends SubsystemBase {

  private CANSparkMax objMotor1 = new CANSparkMax(Constants.canIDs.iRollerMotor1, MotorType.kBrushless); 
  private CANSparkMax objMotor2 = new CANSparkMax(Constants.canIDs.iRollerMotor2, MotorType.kBrushless);

  private double dIntakeSpeed = 0.9;

  /** Creates a new Grabber. */
  public RollerHand_subsystem() {
    objMotor1.setIdleMode(IdleMode.kBrake);
    objMotor2.setIdleMode(IdleMode.kBrake);
    objMotor1.setSmartCurrentLimit(10); // TODO: Make constants
    objMotor2.setSmartCurrentLimit(10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Motor 1 Current", getMotor1Current());
  }

  public double intakeCone(double dSpeed_old) {
    return moveRollers(dIntakeSpeed, dSpeed_old, false);
  }

  public double outtakeCone(double dSpeed_old) {
    // objMotor1.set(-dIntakeSpeed);
    // objMotor2.set(-dIntakeSpeed);
    return moveRollers(-dIntakeSpeed, dSpeed_old, false);
  }

  public double intakeCube(double dSpeed_old) {
    // objMotor1.set(-dIntakeSpeed / 2.0);
    // objMotor2.set(-dIntakeSpeed / 2.0);
    return moveRollers(-dIntakeSpeed / 2.0, dSpeed_old, false);
  }

  public double outtakeCube(double dSpeed_old, boolean bDirection) {
    // bDirection is true for out the top and false for out the bottom (go out same way it came in when intaking)
    if (bDirection) {
      // objMotor1.set(dIntakeSpeed / 2.0);
      // objMotor2.set(-dIntakeSpeed / 2.0);  
      return moveRollers(dIntakeSpeed / 2.0, dSpeed_old, true);
    }
    else {
      // objMotor1.set(-dIntakeSpeed / 2.0);
      // objMotor2.set(dIntakeSpeed / 2.0);
      return moveRollers(-dIntakeSpeed / 2.0, dSpeed_old, true);
    }
  }

  public double getMotor1Current() {
    return objMotor1.getOutputCurrent();
  }

  public double getMotor2Current() {
    return objMotor1.getOutputCurrent();
  }

  public double moveRollers(double dSpeed, double dSpeed_old, boolean bSameDirection) {

    if (Math.abs(dSpeed) > Math.abs(dSpeed_old)) {      //Checking that speed is increasing
      dSpeed = dSpeed_old + Math.min(Math.abs(dSpeed - dSpeed_old), Constants.Wrist.dSpeedUpLimit) * Math.signum(dSpeed);
    }

    // if (Math.abs(dSpeed2) > Math.abs(dSpeed2_old)) {      //Checking that speed is increasing
    //   dSpeed2 = dSpeed2_old + Math.min(Math.abs(dSpeed2 - dSpeed2_old), Constants.Wrist.dSpeedUpLimit) * Math.signum(dSpeed2);
    // }

    if(bSameDirection) {
      objMotor1.set(dSpeed);
      objMotor2.set(-dSpeed);
    }
    else {
      objMotor1.set(dSpeed);
      objMotor2.set(dSpeed);
    }
    
    return dSpeed;
  }

  public void stopMotors() {
    objMotor1.stopMotor();
    objMotor2.stopMotor();
  }
}
