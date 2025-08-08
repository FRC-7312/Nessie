package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoCoralIntake;
import frc.robot.commands.auto.AutoShootCoral;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;

public class ScoreLeft extends SequentialCommandGroup {

  public ScoreLeft(StateMachine stateMachine, Arm arm, Elevator elevator, Swerve swerve, EndEffector endEffector) {

    addCommands(
      new AlignToReefTagRelative("left back", swerve).withTimeout(1),
      new WaitCommand(.5),
      new InstantCommand(() -> stateMachine.requestState(stateMachine.desiredLevel)),
      new WaitCommand(.5),
      new AlignToReefTagRelative("left forward", swerve).withTimeout(1),
      new WaitCommand(1),
      new AutoCoralIntake(endEffector),
      new WaitCommand(.5),
      new AutoShootCoral(endEffector),
      new WaitCommand(.5),
      new AlignToReefTagRelative("left back", swerve).withTimeout(1),
      new WaitCommand(.5),
      new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW))
    );

  }

}
