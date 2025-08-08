package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class IntakeAndHoldAlgae extends Command {

  private EndEffector endEffector;

  public IntakeAndHoldAlgae(EndEffector endEffector) {
    this.endEffector = endEffector;
    addRequirements(endEffector);
  }

  @Override
  public void initialize() {
    endEffector.setVoltage(Constants.EndEffectorConstants.ALGAE_INTAKE_VOLTAGE);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    endEffector.setVoltage(Constants.EndEffectorConstants.ALGAE_HOLD_VOLTAGE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
