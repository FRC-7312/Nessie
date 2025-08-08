package frc.robot.commands.position;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmPosition extends Command {

  private Arm arm;
  private double position;

  public SetArmPosition(Arm arm, double position) {
    this.position = position;
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setPosition(position);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.isFinished();
  }
}