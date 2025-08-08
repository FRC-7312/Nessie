package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class ChangeSpeedMultiplier extends InstantCommand {

  private Swerve swerve;

  public ChangeSpeedMultiplier(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    if(swerve.getSpeedMultiplier() == 1) {
      swerve.setSpeedMultiplier(.25);
    }else {
      swerve.setSpeedMultiplier(1);
    }
  }
  
}
