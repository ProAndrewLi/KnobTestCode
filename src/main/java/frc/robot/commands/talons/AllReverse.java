package frc.robot.commands.talons;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AllReverse extends CommandBase {
  public AllReverse() {
    addRequirements(Robot.MOTOR_TESTING);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Robot.MOTOR_TESTING.first(true); //reversed motor soldering
    Robot.MOTOR_TESTING.second(false);
    Robot.MOTOR_TESTING.third(false); 
    Robot.MOTOR_TESTING.fourth(false);
    Robot.MOTOR_TESTING.fifth(false);
    Robot.MOTOR_TESTING.sixth(false);
    Robot.MOTOR_TESTING.seventh(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      Robot.MOTOR_TESTING.stop();
    }
  }
}
