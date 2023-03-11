package frc.robot.commands.talons;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class All extends CommandBase {
  public All() {
    addRequirements(Robot.MOTOR_TESTING);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Robot.MOTOR_TESTING.first(false); //reversed motor soldering
    Robot.MOTOR_TESTING.second(true);
    Robot.MOTOR_TESTING.third(true);
    Robot.MOTOR_TESTING.fourth(true);
    Robot.MOTOR_TESTING.fifth(true);
    Robot.MOTOR_TESTING.sixth(true);
    Robot.MOTOR_TESTING.seventh(true);
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
