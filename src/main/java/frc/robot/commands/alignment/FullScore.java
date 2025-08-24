package frc.robot.commands.alignment;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.NessieState;
import frc.robot.PositionState;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;

public class FullScore extends SequentialCommandGroup {

  public FullScore(Swerve swerve, StateMachine stateMachine, EndEffector endEffector, PositionState targetPosition, PositionState backPosition, NessieState scoreingState) {
    addRequirements(swerve);

    addCommands(
      new AlignToReef(targetPosition, swerve),
      stateMachine.requestStateCommand(scoreingState),
      new WaitCommand(.3),
      new Approach(swerve, targetPosition),
      endEffector.setVoltageCommand(Constants.EndEffectorConstants.CORAL_SHOOT_VOLTAGE),
      new WaitCommand(.3),
      endEffector.setBrakeCommand(),
      new Approach(swerve, targetPosition),
      stateMachine.requestStateCommand(StateMachine.STOW)
    );
  }
}
