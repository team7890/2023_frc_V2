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
import frc.robot.commands.General_Movement_Commands.Swerve_auto;
// import frc.robot.commands.Roller_commands.ConeIntake_command;
import frc.robot.commands.Roller_commands.ConeOuttake_command;
import frc.robot.commands.Roller_commands.CubeIntake_command;
import frc.robot.commands.General_Movement_Commands.Swerve_balance;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeTopGrabCubeBalance extends SequentialCommandGroup {

  // private boolean bAllianceColor = DriverStation.getAlliance();

  /** Creates a new AutoScoreConeTopAndMove. */
  public ScoreConeTopGrabCubeBalance(Arm_subsystem objArm, Forearm_subsystem objForearm, Wrist_subsystem objWrist, RollerHand_subsystem objRollerHand, Swerve_subsystem objSwerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HiSideScoreConeTop(objArm, objForearm, objWrist).withTimeout(4.0),
      new ConeOuttake_command(objRollerHand).withTimeout(0.5),
      new RegFloorPickupCube(objArm, objForearm, objWrist).withTimeout(1.0),
      new ParallelCommandGroup(
        new Swerve_auto(objSwerve, 0.3, 0.0, 0.0, false).withTimeout(3.0),
        new RegFloorPickupCube(objArm, objForearm, objWrist).withTimeout(3.0),
        new CubeIntake_command(objRollerHand).withTimeout(3.0)
      ).withTimeout(3.0),
      new ParallelCommandGroup(
        new Swerve_auto(objSwerve, 0.2, 0.0, 0.0, false).withTimeout(1.8),
        new RegFloorPickupCube(objArm, objForearm, objWrist).withTimeout(1.8),
        new CubeIntake_command(objRollerHand).withTimeout(1.8)
      ).withTimeout(1.8),
      //below is correct end
      new ParallelCommandGroup(
        new Swerve_balance(objSwerve, -0.45, 0.0, 0.0, false),
        new RegStowArm(objArm, objForearm, objWrist).withTimeout(8.0),
        new CubeIntake_command(objRollerHand).withTimeout(2.0)
      ).withTimeout(9.6)
      // new Swerve_balance(objSwerve, 0.0, 0.0, 0.0, false).withTimeout(0.1),
      // new StowArm(objArm, objForearm, objWrist).withTimeout(1.5),
    );
  }
}
