package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class AutoShootAlgae extends Command {
  
  private EndEffector endEffector;
  private Timer timer;

  public AutoShootAlgae(EndEffector endEffector) {
    this.endEffector = endEffector;
    timer = new Timer();

    addRequirements(endEffector);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    endEffector.setVoltage(Constants.EndEffectorConstants.ALGAE_OUTTAKE_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    endEffector.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > .3;
  }
}