/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ButtonTesting;
import frc.robot.subsystems.MotorTesting;
import frc.robot.subsystems.SolenoidTesting;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot
{
  /*****
   * Robot Objects
   *****/
  public static final RobotMap ROBOTMAP = new RobotMap();
  
  public static final CommandLinker COMMAND_LINKER = new CommandLinker();
  public double count = -0.99;
  
  public static final ButtonTesting BUTTON_TESTING = new ButtonTesting();
  public static final SolenoidTesting SOLENOID_TESTING = new SolenoidTesting();
  public static final MotorTesting MOTOR_TESTING = new MotorTesting();
  //PneumaticsModuleType.CTREPCM is for CTRE Pneumatics Control Module
  //PneumaticsModuleType.REVPH for REV Pneumatics Hub
  public static final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);    
  
  //These are the navX mxp DIO indices
  DigitalInput zero = new DigitalInput(10);
  DigitalInput one = new DigitalInput(11);
  DigitalInput two = new DigitalInput(12);
  DigitalInput three = new DigitalInput(13);
  DigitalInput four = new DigitalInput(18);
  DigitalInput five = new DigitalInput(19);
  DigitalInput six = new DigitalInput(20);
  DigitalInput seven = new DigitalInput(21);
  DigitalInput eight = new DigitalInput(22);
  DigitalInput nine = new DigitalInput(23);

  SendableChooser selection = new SendableChooser();

  String portInput = new String();
  
  @Override
  public void robotInit() 
  {
    Robot.COMMAND_LINKER.configureRegisteredSubsystems();  
    Robot.COMMAND_LINKER.configureCommands();
    compressor.disable();
  }

  @Override
  public void robotPeriodic() {
    if(!zero.get()) portInput = "Blue 1";
    else if(!one.get()) portInput = "Blue 2";
    else if(!two.get()) portInput = "Blue 3";
    else if(!three.get()) portInput = "Blue 4";
    else if(!four.get()) portInput = "Red 1";
    else if(!five.get()) portInput = "Red 2";
    else if(!six.get()) portInput = "Red 3";
    else if(!seven.get()) portInput = "Red 4";
    else portInput = "None";
    SmartDashboard.putString("Knob Value", portInput);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() 
  {
    CommandScheduler.getInstance().enable();
    //MUSIC.playMusic();

  }

  @Override
  public void teleopPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {}
}
