package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.StateMachine;

public class AutoIntake extends Command {

  private StateMachine stateMachine;
  private EndEffector endEffector;
  private Timer timer;

  public AutoIntake(StateMachine stateMachine, EndEffector endEffector) {
    this.stateMachine = stateMachine;
    this.endEffector = endEffector;
    timer = new Timer();

    addRequirements(stateMachine, endEffector);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    endEffector.setVoltage(Constants.EndEffectorConstants.CORAL_INTAKE_VOLTAGE);
    new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW)).schedule();
    new WaitCommand(.2).schedule();
    new InstantCommand(() -> stateMachine.requestState(StateMachine.INTAKE)).schedule();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    new InstantCommand(() -> stateMachine.requestState(StateMachine.INTAKE_CLEARANCE)).schedule();
    new WaitCommand(.2).schedule();
    new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW)).schedule();
    endEffector.setVoltage(Constants.EndEffectorConstants.CORAL_HOLD_VOLTAGE);
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 2;
  }

}