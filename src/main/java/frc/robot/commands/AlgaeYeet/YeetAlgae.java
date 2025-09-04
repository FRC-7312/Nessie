// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeYeet;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.StateMachine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class YeetAlgae extends SequentialCommandGroup {
  /** Creates a new YeetAlgae. */
  public YeetAlgae(StateMachine stateMachine, EndEffector endEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      stateMachine.requestStateCommand(StateMachine.ALGAE_PRE_YEET),
      new WaitCommand(.5),
      stateMachine.requestStateCommand(StateMachine.ALGAE_YEET).andThen(new WaitCommand(.067)),
      endEffector.setVoltageCommand(Constants.EndEffectorConstants.ALGAE_SHOOT_VOLTAGE),
      new WaitCommand(.5),
      endEffector.setBrakeCommand(),
      stateMachine.requestStateCommand(StateMachine.STOW)
    );
  }
}
