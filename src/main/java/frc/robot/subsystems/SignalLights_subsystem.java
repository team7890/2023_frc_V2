// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Our Imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class SignalLights_subsystem extends SubsystemBase {
  
  private AddressableLED objLED;
  private AddressableLEDBuffer objLEDBufferYellow;
  private AddressableLEDBuffer objLEDBufferPurple;
  private AddressableLEDBuffer objLEDBufferGreen;
  private AddressableLEDBuffer objLEDBufferOff;
  private String sColor = "OFF";
  private int iLength = 100;

  
  /** Creates a new SignalLights_subsystem. */
  public SignalLights_subsystem() {
    objLED = new AddressableLED(5);
    objLEDBufferYellow = new AddressableLEDBuffer(iLength);
    objLEDBufferPurple = new AddressableLEDBuffer(iLength);
    objLEDBufferGreen = new AddressableLEDBuffer(iLength);
    objLEDBufferOff = new AddressableLEDBuffer(iLength);
    objLED.setLength(objLEDBufferOff.getLength());

    for (var i = 0; i < objLEDBufferOff.getLength(); i++) {
      objLEDBufferYellow.setRGB(i, 30, 30, 0);
      objLEDBufferPurple.setRGB(i, 30, 0, 30);
      objLEDBufferGreen.setRGB(i,0,255,0);
      objLEDBufferOff.setRGB(i, 0, 0, 0);
    }
    objLED.setData(objLEDBufferOff);
    objLED.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("LED Color", sColor);
  }

  public CommandBase changeLightColor() {
    return runOnce(
      () -> { 
        switch(sColor){
          case "OFF":
            sColor = "YELLOW";
            objLED.setData(objLEDBufferYellow);
            objLED.start();
            break;
          case "YELLOW":
            sColor = "PURPLE";
            objLED.setData(objLEDBufferPurple);
            objLED.start();
            break;
          case "PURPLE":
            sColor = "GREEN";
            objLED.setData(objLEDBufferGreen);
            objLED.start();
            break;
          case "GREEN":
            sColor = "OFF";
            objLED.setData(objLEDBufferOff);
            objLED.start();
            break;
      }
      }
    );
  }
}
