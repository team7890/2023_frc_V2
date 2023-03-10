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

  private double dIntakeSpeed = 0.3;
  private double dSpeedUpLimit = 0.03;

  /** Creates a new Grabber. */
  public RollerHand_subsystem() {
    objMotor1.setIdleMode(IdleMode.kBrake);
    objMotor2.setIdleMode(IdleMode.kBrake);
    objMotor1.setSmartCurrentLimit(5); // TODO: Make constants
    objMotor2.setSmartCurrentLimit(5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Motor 1 Current", getMotor1Current());
    dSpeedUpLimit = SmartDashboard.getNumber("Roller Speed Up Limit", dSpeedUpLimit);
  }

  public double intakeCone(double dSpeed_old) {
    return moveRollers(dIntakeSpeed, dSpeed_old, false);
  }

  public void intakeConeTester() {
    double dSpeed = 0.5;
    objMotor1.set(dSpeed);
    objMotor2.set(dSpeed);
  }

  public void outtakeCone() {
    double dSpeed = 0.5;
    objMotor1.set(-dSpeed);
    objMotor2.set(-dSpeed);
    // return moveRollers(-dIntakeSpeed, dSpeed_old, false);
  }

  public void intakeCube() {
    double dSpeed = 0.5;
    objMotor1.set(-dSpeed);
    objMotor2.set(-dSpeed);
  }

  public void outtakeCube(boolean bDirection) {
    // bDirection is true for out the top and false for out the bottom (go out same way it came in when intaking)
    double dSpeed = 0.5;
    if (bDirection) {
      objMotor1.set(dSpeed);
      objMotor2.set(-dSpeed);
    }
    else {
      objMotor1.set(-dSpeed);
      objMotor2.set(dSpeed);
    }
  }

  public double getMotor1Current() {
    return objMotor1.getOutputCurrent();
  }

  public double getMotor2Current() {
    return objMotor1.getOutputCurrent();
  }
  
  public double getSpeed() {
    return objMotor1.get();
  }

  public double moveRollers(double dSpeed, double dSpeed_old, boolean bSameDirection) {

    if (Math.abs(dSpeed) > Math.abs(dSpeed_old)) {      //Checking that speed is increasing
      dSpeed = dSpeed_old + Math.min(Math.abs(dSpeed - dSpeed_old), dSpeedUpLimit) * Math.signum(dSpeed);
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
