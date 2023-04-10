// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// our Imports
import frc.robot.subsystems.Arm_subsystem;
import frc.robot.subsystems.Forearm_subsystem;
import frc.robot.subsystems.Wrist_subsystem;
import frc.robot.subsystems.Swerve_subsystem;
import frc.robot.subsystems.RollerHand_subsystem;
import frc.robot.commands.Button_commands.Regular_Side_Commands.*;
import frc.robot.commands.Button_commands.High_Side_Commands.*;
import frc.robot.commands.General_Movement_Commands.Swerve_AutoTurnToZeroDegrees;
import frc.robot.commands.General_Movement_Commands.Swerve_auto;
import frc.robot.commands.Roller_commands.ConeIntake_command;
// import frc.robot.commands.Roller_commands.ConeIntake_command;
import frc.robot.commands.Roller_commands.ConeOuttake_command;
import frc.robot.commands.Roller_commands.CubeIntake_command;
import frc.robot.commands.General_Movement_Commands.Swerve_BingoBongo_Balance;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BingoBongo extends SequentialCommandGroup {
//Score Cone Top, Over Balance, Grab Horz Cone, Over Balance, Score Cone middle, Balance

  // private boolean bAllianceColor = DriverStation.getAlliance();

  /** Creates a new AutoScoreConeTopAndMove. */
  public BingoBongo(Arm_subsystem objArm, Forearm_subsystem objForearm, Wrist_subsystem objWrist, RollerHand_subsystem objRollerHand, Swerve_subsystem objSwerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new Swerve_auto(objSwerve, -0.3, 0.0, 0.0, false).withTimeout(0.1),
        new HiSideScoreConeTop(objArm, objForearm, objWrist).withTimeout(4.0)
      ).withTimeout(4.0),
      new ConeOuttake_command(objRollerHand).withTimeout(0.5),
      new RegFloorPickupHorzCone(objArm, objForearm, objWrist).withTimeout(0.5),
      new ParallelCommandGroup(
        new Swerve_auto(objSwerve, 0.4, 0.0, 0.0, false).withTimeout(3.25),
        new RegFloorPickupHorzCone(objArm, objForearm, objWrist).withTimeout(3.25),
        new ConeIntake_command(objRollerHand).withTimeout(3.25)
      ).withTimeout(3.0),
      new ParallelCommandGroup(
        new Swerve_auto(objSwerve, 0.15, 0.0, 0.0, false).withTimeout(3.0),
        new RegFloorPickupHorzCone(objArm, objForearm, objWrist).withTimeout(3.0),
        new ConeIntake_command(objRollerHand).withTimeout(3.0)
      ).withTimeout(1.2),   // To here works and gets cone
      new ParallelCommandGroup(
        new Swerve_auto(objSwerve, -0.4, 0.0, 0.0, false).withTimeout(3.0),
        new RegScoreConeLow(objArm, objForearm, objWrist).withTimeout(3.0),
        new ConeIntake_command(objRollerHand).withTimeout(0.25)
      ).withTimeout(2.9),
      new ParallelCommandGroup(
        new Swerve_AutoTurnToZeroDegrees(objSwerve).withTimeout(3.0),
        new RegStowArm(objArm, objForearm, objWrist).withTimeout(3.0),
        new ConeIntake_command(objRollerHand).withTimeout(0.2)
      ).withTimeout(1.0),
      new ParallelCommandGroup(
        new Swerve_auto(objSwerve, -0.6, 0.0, 0.0, false).withTimeout(3.0),
        new RegStowArm(objArm, objForearm, objWrist).withTimeout(3.0),
        new ConeOuttake_command(objRollerHand).withTimeout(0.25)
      ).withTimeout(0.25), //below is correct end
      new ParallelCommandGroup(
        new Swerve_BingoBongo_Balance(objSwerve, 0.3, 0.0, 0.0, false),
        new RegStowArm(objArm, objForearm, objWrist).withTimeout(8.0)
      ).withTimeout(9.6)
    );
  }
}
