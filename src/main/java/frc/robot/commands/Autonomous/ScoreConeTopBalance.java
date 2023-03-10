// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// our Imports
import frc.robot.subsystems.Arm_subsystem;
import frc.robot.subsystems.Forearm_subsystem;
import frc.robot.subsystems.Wrist_subsystem;
import frc.robot.subsystems.xGrabber_subsystem;
import frc.robot.subsystems.Swerve_subsystem;

import frc.robot.commands.xGrabber_command;
import frc.robot.commands.Swerve_balance;
import frc.robot.commands.Button_commands.ScoreConeTop3;
import frc.robot.commands.Button_commands.StowArm;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeTopBalance extends SequentialCommandGroup {

  // private boolean bAllianceColor = DriverStation.getAlliance();

  /** Creates a new AutoScoreConeTopAndMove. */
  public ScoreConeTopBalance(Arm_subsystem objArm, Forearm_subsystem objForearm, Wrist_subsystem objWrist, xGrabber_subsystem objGrabber, Swerve_subsystem objSwerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreConeTop3(objArm, objForearm, objWrist).withTimeout(5.3),
      new xGrabber_command(objGrabber).withTimeout(0.1),
      new StowArm(objArm, objForearm, objWrist).withTimeout(1.0),
      new ParallelCommandGroup(
        new Swerve_balance(objSwerve, 0.3, 0.0, 0.0, false),
        new SequentialCommandGroup(
          new StowArm(objArm, objForearm, objWrist).withTimeout(8.0),
          new xGrabber_command(objGrabber).withTimeout(0.1)
        )
      ).withTimeout(9.6)
      // new Swerve_balance(objSwerve, 0.0, 0.0, 0.0, false).withTimeout(0.1),
      // new StowArm(objArm, objForearm, objWrist).withTimeout(1.5),
    );
  }
}
