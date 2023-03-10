package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

import java.lang.Math;

/* Testing Position Idea */
// import frc.robot.commands.*;
// import frc.robot.subsystems.*;

public class Utilities {

    public static double correctAngle(double dEncoder_in, double dOffset_in, double dDegreesPerRev_in) {
        double dAngle;
        dAngle = dEncoder_in;
        SmartDashboard.putNumber("value", dAngle);
        
        dAngle = (dAngle * dDegreesPerRev_in);   
        dAngle = dAngle - dOffset_in;       //makes the percents aligned with the table horizontally
        dAngle = dAngle % 360.0;
        if ((dAngle % 360.0) < 0) {
          dAngle = dAngle + 360.0;
        }
        SmartDashboard.putNumber("Angle", dAngle);
        return dAngle - 180.0;
    }

    // correctAngle2 generates a corrected mechanism angle from an encoder input
    //      dEncoder_in is the encoder input, with a full revolution changing the sensor's reading by 1.0
    //          in other words, if 12 o'clock is 0.0 then rotating to 12 o'clock again will result in 1.0 and again 2.0
    //          and rotating in the other direction from 12 o'clock would result in -1.0 and again -2.0
    //      dOffset_in is the offset angle which is the resulting angle from the encoder 
    //          through this correctAngle2 calculation when the mechanism is in the 0 degree position
    //          and when dOffset_in is set to 0.0
    //      dRatio is the ratio of encoder revolutions to mechanism revolutions, or the gear ratio from
    //          mechanism to encoder (if the encoder has an 18T pulley and the wrist has a 36T pulle, the ratio would be 2.0
    //      bInvert is used to invert the rotation angle of the encoder
    //          it should be true to reverse the encoder angle so that + motor speed results in increasing angle
    //          so that control loop logic works correctly
    //      dAngle is the returned value which is between -180.0 and +180.0
    public static double correctAngle2(double dEncoder_in, double dOffset_in, double dRatio, boolean bInvert) {
        double dAngle;
        double dDegreesPerRev = 360.0 / dRatio;     // degrees of mechanism motion in one encoder revolution

        // dAngle is computed by taking the encoder reading and subtracting the offset angle which is
        //      back-converted to encoder units, then taking the modulus of this result relative to the ratio
        //      to obtain periodicity to one revolution of the wrist part of the mechanism, in terms of revs of the encoder.
        //      This result is multiplied by dDegreesPerRev to get revs of the wrist mechanism in degrees.
        //      Assuming the encoder does not wrap more than once, a just-in-case correction is to add 360
        //      if the angle is less than -180 and subtract 360 if the angle is more than +180 so that the result
        //      is scaled to between -180 and +180 degrees.

        if (bInvert) {
            dAngle = ((dEncoder_in + dOffset_in / dDegreesPerRev) % dRatio) * dDegreesPerRev;
        }
        else {
            dAngle = ((dEncoder_in - dOffset_in / dDegreesPerRev) % dRatio) * dDegreesPerRev;
        }
        if (dAngle < -180.0) dAngle = dAngle + 360.0;
        if (dAngle > 180.0) dAngle = dAngle - 360.0;
        if (bInvert) dAngle = -dAngle;
        return dAngle;
    }

    public static double limitVariable(double dMinValue, double dVariable, double dMaxValue) {
        double dValue;
        dValue = Math.max(dVariable, dMinValue);
        dValue = Math.min(dValue, dMaxValue);        
        return dValue;
    }

    /* Testing Position Idea */
    // private final Arm_subsystem objArm_subsystem = new Arm_subsystem();
    // private final Forearm_subsystem objForearm_subsystem = new Forearm_subsystem();
    // private final Wrist_subsystem objWrist_subsystem = new Wrist_subsystem();

    // public Command PickupPosition() {
    //     return new Arm_command(objArm_subsystem, -Constants.Arm.dArmSpeedManual, true, -10.5),                                //Doesn't like negative?
    //     new Forearm_command(objForearm_subsystem, -Constants.Forearm.dForearmSpeedManual, true, 117.52),
    //     new Wrist_command(objWrist_subsystem, -Constants.Wrist.dWristSpeedManual, true, 12.85);
    // }
} 
