package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class ShootCoral extends Command {

  private EndEffector endEffector;

  public ShootCoral(EndEffector endEffector) {
    this.endEffector = endEffector;
    addRequirements(endEffector);
  }

  @Override
  public void initialize() {
    endEffector.setVoltage(Constants.EndEffectorConstants.CORAL_OUTTAKE_VOLTAGE);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    endEffector.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
