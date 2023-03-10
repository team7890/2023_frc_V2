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



public class CoolLedTricks_Subsystem extends SubsystemBase {
  
  private AddressableLED objLED;
  private AddressableLEDBuffer objLEDBufferGreen;
//   private AddressableLEDBuffer objLEDBufferPurple;
  private AddressableLEDBuffer objLEDBufferOff;
  private String sColor = "OFF";
  

  
  /** Creates a new CoolLedTricks_Subsystem. */
  public CoolLedTricks_Subsystem() {
    objLED = new AddressableLED(4);
    objLEDBufferGreen = new AddressableLEDBuffer(16);
    // objLEDBufferPurple = new AddressableLEDBuffer(16);
    objLEDBufferOff = new AddressableLEDBuffer(16);
    objLED.setLength(objLEDBufferOff.getLength());

    for (var i = 0; i < objLEDBufferOff.getLength(); i++) {
      for (int iGreenNumVal = 130; iGreenNumVal <= 255; iGreenNumVal++) {                       //Makes the green value shift from 30 to 255
        objLEDBufferGreen.setRGB(i, 0, iGreenNumVal, 0);
      }
      for (var iGreenNumVal = 254; iGreenNumVal > 130; iGreenNumVal++) {
        objLEDBufferGreen.setRGB(i, 0, iGreenNumVal, 0);
      }
    //   objLEDBufferPurple.setRGB(i, 30, 0, 30);
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
            objLED.setData(objLEDBufferGreen);
            objLED.start();
            break;
          case "GreenShift":
            sColor = "OFF";
            objLED.setData(objLEDBufferOff);
            objLED.start();
            break;
        }
      }
    );
  }
}
