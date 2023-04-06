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
  private AddressableLEDBuffer objLEDBufferChase;
  private String sColor = "OFF";
  private int iLength = 80;
  private int iStartPosition;   // varying start position for chasing lights
  private double dStartPosition;
  private int iLightPosition;   // position in the string to set light in loop with start position accounted for
  private double dSpeedCoefficient = 2.5;   // higher makes lights go faster
  
  /** Creates a new SignalLights_subsystem. */
  public SignalLights_subsystem() {
    objLED = new AddressableLED(5);
    objLEDBufferYellow = new AddressableLEDBuffer(iLength);
    objLEDBufferPurple = new AddressableLEDBuffer(iLength);
    objLEDBufferGreen = new AddressableLEDBuffer(iLength);
    objLEDBufferOff = new AddressableLEDBuffer(iLength);
    objLEDBufferChase = new AddressableLEDBuffer(iLength);
    objLED.setLength(objLEDBufferOff.getLength());

    for (var i = 0; i < objLEDBufferOff.getLength(); i++) {
      objLEDBufferYellow.setRGB(i, 30, 30, 0);
      objLEDBufferPurple.setRGB(i, 30, 0, 30);
      objLEDBufferGreen.setRGB(i, 0, 130, 0);
      objLEDBufferOff.setRGB(i, 0, 0, 0);
    }
    // objLED.setData(objLEDBufferOff);
    // objLED.start();
    makeNateHappy();
    iStartPosition = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putString("LED Color", sColor);
    // makeNateHappy();
  }

  public void chaseLights(double dSpeed) {
    for (var iLoop = 0; iLoop < iLength / 2; iLoop++) {
      final var iGreen = 130; // * iLoop / 10;
      iLightPosition = iStartPosition + iLoop;
      if (iLightPosition > 49) iLightPosition = iLightPosition - 50;
      if (iLoop < 10) {
        objLEDBufferChase.setRGB(iLightPosition, 0, iGreen, 0);
        objLEDBufferChase.setRGB(78 - iLightPosition, 0, iGreen, 0);
      }
      else {
        objLEDBufferChase.setRGB(iLightPosition, 0, 0, 0);
        objLEDBufferChase.setRGB(78 - iLightPosition, 0, 0, 0);
      }
    }
    objLED.setData(objLEDBufferChase);
    objLED.start();

    if (Math.abs(dSpeed) > 0.05) {
      dStartPosition = dStartPosition + dSpeed * dSpeedCoefficient;
    }

    // iStartPosition = iStartPosition + 1;
    // if (iStartPosition > 49) iStartPosition = 0;
    // if (iStartPosition < 0) iStartPosition = 49;
    if (dStartPosition > 49.4) dStartPosition = 0.0;
    if (dStartPosition < -0.4) dStartPosition = 49.0;
    iStartPosition = (int)dStartPosition;
  }

  public void turnLightsOff() {
    objLED.setData(objLEDBufferOff);
    objLED.start();
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

  public void makeNateHappy() {
    objLED.setData(objLEDBufferGreen);
    objLED.start();
  }
}
