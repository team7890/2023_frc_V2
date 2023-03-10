// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.Wrist;
// import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// //Arm Imports
// import frc.robot.subsystems.Arm_subsystem;
// //ForeArm Imports
// import frc.robot.subsystems.Forearm_subsystem;
// //Wrist Imports
// import frc.robot.subsystems.Wrist_subsystem;
//Begginning of Swerve Imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//Already Imported: import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.command_groups.*;
import frc.robot.commands.*;
import frc.robot.commands.Autonomous.AutoScoreConeTop;
import frc.robot.commands.Autonomous.ScoreConeTopMoveLong;
import frc.robot.commands.Autonomous.ScoreConeTopMoveShort;
import frc.robot.commands.Autonomous.ScoreConeTopBalance;
import frc.robot.commands.Button_commands.*;
import frc.robot.commands.Roller_commands.ConeIntake;
import frc.robot.commands.Roller_commands.ConeOuttake;
import frc.robot.commands.Roller_commands.CubeIntake;
import frc.robot.commands.Roller_commands.CubeOuttake;
import frc.robot.subsystems.*;
//End of Swerve Imports

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Arm_subsystem objArm_subsystem = new Arm_subsystem();
  private final Forearm_subsystem objForearm_subsystem = new Forearm_subsystem();
  private final Wrist_subsystem objWrist_subsystem = new Wrist_subsystem();
  // private final SignalLights_subsystem objSignalLights_subsystem = new SignalLights_subsystem();
  private final xGrabber_subsystem objGrabber_subsystem = new xGrabber_subsystem();

  private final RollerHand_subsystem objRollerHand = new RollerHand_subsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_CoPilotController = new CommandXboxController(Constants.Controllers.iCoPilot);
  
  private final Joystick obj_ButtonBox = new  Joystick(Constants.Controllers.iButtonBox);
  JoystickButton ButtonOne = new JoystickButton(obj_ButtonBox, 1);    // Top left
  JoystickButton ButtonTwo = new JoystickButton(obj_ButtonBox, 2);    // Middle Left
  JoystickButton ButtonThree = new JoystickButton(obj_ButtonBox, 3);  // Bottom Left
  JoystickButton ButtonFour = new JoystickButton(obj_ButtonBox, 4);   // Top Right
  JoystickButton ButtonFive = new JoystickButton(obj_ButtonBox, 5);   // Middle Right
  JoystickButton ButtonSix = new JoystickButton(obj_ButtonBox, 6);    // Bottom Right
  JoystickButton ButtonSeven = new JoystickButton(obj_ButtonBox, 7);    // Bottom Right
  JoystickButton ButtonEight = new JoystickButton(obj_ButtonBox, 8);    // Middle Bottom

  

  //For Swerve
  /* Controllers */
  // private final Joystick m_DriverController = new Joystick(Constants.Controllers.iDriver);            //This is a change from team:364 code because we used CommandXboxController
  private final CommandXboxController m_DriverController = new CommandXboxController(Constants.Controllers.iDriver);    //Amalan How do we do this with CommandXboxController?

  /* Drive Controls */
  // private final int translationAxis = XboxController.Axis.kLeftY.value;            //This is a change from team:364 code because we used CommandXboxController
  // private final int strafeAxis = XboxController.Axis.kLeftX.value;            //This is a change from team:364 code because we used CommandXboxController
  // private final int rotationAxis = XboxController.Axis.kRightX.value;            //This is a change from team:364 code because we used CommandXboxController

  /* Driver Buttons */
  // private final JoystickButton zeroGyro = new JoystickButton(m_DriverController, XboxController.Button.kY.value);            //This is a change from team:364 code because we used CommandXboxController
  // private final JoystickButton robotCentric = new JoystickButton(m_DriverController, XboxController.Button.kLeftBumper.value);            //This is a change from team:364 code because we used CommandXboxController

  /* Subsystems */
  private final Swerve_subsystem objSwerve_subsystem = new Swerve_subsystem();

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final SequentialCommandGroup m_ScoreConeTopMoveShort = new ScoreConeTopMoveShort(objArm_subsystem, objForearm_subsystem, objWrist_subsystem, objGrabber_subsystem, objSwerve_subsystem);
  private final SequentialCommandGroup m_ScoreConeTopMoveLong = new ScoreConeTopMoveLong(objArm_subsystem, objForearm_subsystem, objWrist_subsystem, objGrabber_subsystem, objSwerve_subsystem);
  private final SequentialCommandGroup m_ScoreConeTopBalance = new ScoreConeTopBalance(objArm_subsystem, objForearm_subsystem, objWrist_subsystem, objGrabber_subsystem, objSwerve_subsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Swerve Stuff
    
          //This is a change from team:364 code because we used CommandXboxController
    // s_Swerve.setDefaultCommand(              //Amalan How do we do this with CommandXboxController?
    //   new TeleopSwerve(
    //     s_Swerve, 
        // () -> -m_DriverController.getRawAxis(translationAxis), 
    //     () -> -m_DriverController.getRawAxis(strafeAxis), 
    //     () -> -m_DriverController.getRawAxis(rotationAxis), 
    //     () -> robotCentric.getAsBoolean()
    //   )
    // );

    objSwerve_subsystem.setDefaultCommand(
      new Swerve_teleop(
        objSwerve_subsystem, 
        () -> -m_DriverController.getLeftY(), 
        () -> -m_DriverController.getLeftX(),
        () -> -m_DriverController.getRightX(),
        // () -> m_DriverController.leftBumper().getAsBoolean()
        () -> false,     // Never using Robot Centric Mode
        () -> m_DriverController.leftBumper().getAsBoolean()
      )
    );


    
    // Configure the trigger bindings
    configureBindings();

    m_chooser.setDefaultOption("ScoreConeTop MoveShort", m_ScoreConeTopMoveShort);
    m_chooser.addOption("ScoreConeTop MoveLong", m_ScoreConeTopMoveLong);
    m_chooser.addOption("ScoreConeTop Balance", m_ScoreConeTopBalance);


    // m_chooser.addOption("Complex Auto", m_complexAuto);
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_CoPilotController.x().whileTrue(new Arm_command(objArm_subsystem, Constants.Arm.dArmSpeedManual, false, 0.0));         
    m_CoPilotController.y().whileTrue(new Arm_command(objArm_subsystem, -Constants.Arm.dArmSpeedManual, false, 0.0));        
    // m_CoPilotController.rightBumper().whileTrue(new Arm_command(objArm_subsystem, -Constants.Arm.dArmSpeedManual, true, -16.7));                    //Ask about - infront of Constants.Arm.dArmSpeedManual 

    m_CoPilotController.back().whileTrue(new Forearm_command(objForearm_subsystem, Constants.Forearm.dSpeedManual, false, 0.0));           // Back Button = Left button
    m_CoPilotController.start().whileTrue(new Forearm_command(objForearm_subsystem, -Constants.Forearm.dSpeedManual, false, 0.0));         // Start Button = Right button
    // m_CoPilotController.leftBumper().whileTrue(new Forearm_command(objForearm_subsystem, -Constants.Forearm.dSpeedManual, true, 0.0));     //Also has the negative -
    
    m_CoPilotController.a().whileTrue(new Wrist_command(objWrist_subsystem, -Constants.Wrist.dSpeedManual, false, 0.0));
    m_CoPilotController.b().whileTrue(new Wrist_command(objWrist_subsystem, Constants.Wrist.dSpeedManual, false, 0.0));
    // m_CoPilotController.rightTrigger().whileTrue(new Wrist_command(objWrist_subsystem, -Constants.Wrist.dWristSpeedManual, true, 0.0));       //Also has the negative -

    // m_CoPilotController.rightBumper().whileTrue(new STEAL(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));
    
    
    
    
    
    /* ButtonBox Stuff */


    // Left Side of Button box (Top to Bottom)
    ButtonFour.whileTrue(new ScoreCubeTop2(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));    //Testing new version of ScoreCubeTop
    ButtonFive.whileTrue(new ScoreCubeMiddle(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));
    // ButtonSix.whileTrue(new StowArm(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));
    ButtonSix.whileTrue(new GeneralPickup(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));

    // ButtonSix.whileTrue(new DoubleSubstationPickup(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));

    // Right Side of Button box (Top to Bottom)    
    ButtonOne.whileTrue(new ScoreConeTop3(objArm_subsystem, objForearm_subsystem, objWrist_subsystem)); 
    ButtonTwo.whileTrue(new ScoreConeMiddle2(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));
    ButtonThree.whileTrue(new PickupVerticalCone(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));

    

    // Middle buttons (Top and Bottom)    
    ButtonSeven.whileTrue(new DoubleSubstationPickup(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));
    ButtonEight.whileTrue(new StowArm(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));
    // ButtonSeven.whileTrue(new Mech_command(objArm_subsystem, objForearm_subsystem, objWrist_subsystem, 0.0, 0.0, 0.0));
    // ButtonEight.debounce(0.05).onTrue(new Grabber_command(objGrabber_subsystem));
    
    // Currently Unused Buttons
    // ButtonSix.whileTrue(new ScoreCubeMiddle(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));
    // ButtonSix.whileTrue(new ScoreBottom(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));
    // ButtonOne.onTrue(objSignalLights_subsystem.changeLightColor());


    // Currently Working on this button
    // ButtonThree.whileTrue(new xScoreConeMiddleOld(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));
    // 





    // Open/ Close grabber
    // m_DriverController.a().debounce(0.05).onTrue(new Grabber_command(objGrabber_subsystem));
    m_DriverController.rightBumper().whileTrue(new StowArm(objArm_subsystem, objForearm_subsystem, objWrist_subsystem));
    // leftbumper will be slow mode


    /* Driver Buttons */    //For Swerve
    // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));            //This is a change from team:364 code because we used CommandXboxController
    m_DriverController.back().onTrue(new InstantCommand(objSwerve_subsystem::zeroGyro, objSwerve_subsystem));                                  //Amalan How do we do this with CommandXboxController?


    m_DriverController.leftTrigger().whileTrue(new ConeIntake(objRollerHand));
    m_DriverController.rightTrigger().whileTrue(new ConeOuttake(objRollerHand));
    m_DriverController.a().whileTrue(new CubeIntake(objRollerHand));
    m_DriverController.x().whileTrue(new CubeOuttake(objRollerHand, false));
    m_DriverController.b().whileTrue(new CubeOuttake(objRollerHand, true));


    // try with xbox controller but commented out when got button box above working
    // m_DriverController.a().onTrue(objSignalLights_subsystem.changeLightColor());

    // added to test autos with button because getAutoCmd was not working
    // m_DriverController.start().whileTrue(new ScoreConeTopBalance(objArm_subsystem, objForearm_subsystem, objWrist_subsystem, objGrabber_subsystem, objSwerve_subsystem));
    // m_DriverController.leftBumper().whileTrue(new Swerve_balance(objSwerve_subsystem, .25, 0, 0, false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //   // An example command will be run in autonomous
    //   // return Autos.exampleAuto(m_exampleSubsystem);
  
    // An ExampleCommand will run in autonomous

    // return new ScoreConeTopMoveShort(objArm_subsystem, objForearm_subsystem, objWrist_subsystem, objGrabber_subsystem, s_Swerve);

    return m_chooser.getSelected();



    // return new exampleAuto(s_Swerve);
  }


  
}
