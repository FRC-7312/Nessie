package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.LimelightHelpers;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.alignment.AlignToReef;
import frc.robot.commands.alignment.Approach;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;

@SuppressWarnings("unused")
public class RobotContainer {

    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.ControlConstants.DRIVER_PORT);
    public final CommandXboxController operator = new CommandXboxController(Constants.ControlConstants.OPERATOR_PORT);
    private final GenericHID operatorKeypad = new GenericHID(Constants.ControlConstants.OPERATOR_KEYPAD_PORT);

    /* Drive Controls */
    private final int leftY = XboxController.Axis.kLeftY.value;
    private final int leftX = XboxController.Axis.kLeftX.value;
    private final int rightX = XboxController.Axis.kRightX.value;

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final EndEffector endEffector = new EndEffector();
    private final StateMachine stateMachine = new StateMachine(elevator, arm);

    /* Autos */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        /* Command Configuration */
        NamedCommands.registerCommand("Stow Position", stateMachine.requestStateCommand(StateMachine.STOW));
        NamedCommands.registerCommand("Coast Position", stateMachine.requestStateCommand(StateMachine.COAST));
        NamedCommands.registerCommand("L1 Position", stateMachine.requestStateCommand(StateMachine.L1));
        NamedCommands.registerCommand("L2 Position", stateMachine.requestStateCommand(StateMachine.L2));
        NamedCommands.registerCommand("L3 Position", stateMachine.requestStateCommand(StateMachine.L3));
        NamedCommands.registerCommand("L4 Position", stateMachine.requestStateCommand(StateMachine.L4));
        NamedCommands.registerCommand("Intake", new SequentialCommandGroup(
            stateMachine.requestStateCommand(StateMachine.STOW),
            new WaitCommand(.2),
            stateMachine.requestStateCommand(StateMachine.INTAKE),
            endEffector.setVoltageCommand(Constants.EndEffectorConstants.CORAL_INTAKE_VOLTAGE),
            new WaitCommand(1),
            endEffector.setVoltageCommand(Constants.EndEffectorConstants.CORAL_HOLD_VOLTAGE),
            stateMachine.requestStateCommand(StateMachine.STOW)
        ));

        SmartDashboard.putData("Align to Reef Right", new AlignToReef(StateMachine.RIGHT_FORWARD, swerve));
        SmartDashboard.putData("Align to Reef Left ", new AlignToReef(StateMachine.LEFT_FRONT, swerve));

        
        SmartDashboard.putData("Approach Left", new Approach(swerve, StateMachine.LEFT_FRONT));
        SmartDashboard.putData("Approach Right", new Approach(swerve, StateMachine.RIGHT_FORWARD));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Selected Auto", autoChooser);

        configureButtonBindings();

    }

    private void configureButtonBindings() {

        /* Driver */

        // driver left y to move forward/backward
        // driver left x to strafe left/right
        // driver right x to rotate left/right
        // driver left bumper press to x lock
        // driver right bumper press to drive robot-centric
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve,
                () -> -MathUtil.applyDeadband(driver.getRawAxis(leftY), Constants.ControlConstants.STICK_DEADBAND),
                () -> -MathUtil.applyDeadband(driver.getRawAxis(leftX), Constants.ControlConstants.STICK_DEADBAND),
                () -> -MathUtil.applyDeadband(driver.getRawAxis(rightX), Constants.ControlConstants.STICK_DEADBAND),
                () -> driver.leftBumper().getAsBoolean(),
                () -> driver.rightBumper().getAsBoolean(),
                () -> driver.povRight().getAsBoolean()
            )
        );

        // driver left bumper to align to reef on the left
        // driver right bumper to align to reef on the right
        driver.rightBumper().onTrue(new AlignToReef(StateMachine.RIGHT_FORWARD, swerve));
        driver.leftBumper().onTrue(new AlignToReef(StateMachine.LEFT_FRONT, swerve));

        // driver pov up to swap from fast mode to slow mode
        driver.povUp().onTrue(swerve.changeSpeedMultiplierCommand());
        // driver pov right to zero heading for swerve
        driver.povDown().onTrue(swerve.zeroHeading());
        

        // driver left trigger to intake algae, and hold it when released
        driver.leftTrigger().whileTrue(endEffector.setVoltageCommand(Constants.EndEffectorConstants.ALGAE_INTAKE_VOLTAGE));
        driver.leftTrigger().onFalse(endEffector.setVoltageCommand(Constants.EndEffectorConstants.ALGAE_HOLD_VOLTAGE));

        // driver right trigger to shoot algae
        driver.rightTrigger().onTrue(endEffector.setVoltageCommand(Constants.EndEffectorConstants.ALGAE_SHOOT_VOLTAGE));
        driver.rightTrigger().onFalse(endEffector.setVoltageCommand(0));

        // driver a to request algae taxi state
        // driver b to request algae intake low state
        // driver y to request algae intake high state
        // driver x to request algae process state
        driver.a().onTrue(stateMachine.requestStateCommand(StateMachine.ALGAE_TAXI));
        driver.b().onTrue(stateMachine.requestStateCommand(StateMachine.ALGAE_INTAKE_LOW));
        driver.y().onTrue(stateMachine.requestStateCommand(StateMachine.ALGAE_INTAKE_HIGH));
        driver.x().onTrue(stateMachine.requestStateCommand(StateMachine.ALGAE_PROCESS));

        /* Operator */

        // operator a to request L1 state
        // operator b to request L2 state
        // operator y to request L3 state
        // operator x to request L4 state
        operator.a().onTrue(stateMachine.requestStateCommand(StateMachine.L1));
        operator.b().onTrue(stateMachine.requestStateCommand(StateMachine.L2));
        operator.y().onTrue(stateMachine.requestStateCommand(StateMachine.L3));
        operator.x().onTrue(stateMachine.requestStateCommand(StateMachine.L4));

        // operator left bumper press to intake coral
        operator.leftBumper().onTrue(
            new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW))
            .andThen(new WaitCommand(.2)).andThen(() -> stateMachine.requestState(StateMachine.INTAKE))
        );
        operator.leftBumper().whileTrue(endEffector.setVoltageCommand(Constants.EndEffectorConstants.CORAL_INTAKE_VOLTAGE));
        operator.leftBumper().onFalse(endEffector.setVoltageCommand(Constants.EndEffectorConstants.CORAL_HOLD_VOLTAGE)
            .andThen(stateMachine.requestStateCommand(StateMachine.INTAKE_CLEARANCE))
            .andThen(new WaitCommand(.2)).andThen(() -> stateMachine.requestState(StateMachine.STOW))
        );

        // operator left trigger to stow
        operator.leftTrigger().onTrue(stateMachine.requestStateCommand(StateMachine.STOW));

        // operator right trigger to shoot coral
        operator.rightTrigger().onTrue(endEffector.setVoltageCommand(Constants.EndEffectorConstants.CORAL_SHOOT_VOLTAGE));
        operator.rightTrigger().onFalse(endEffector.setVoltageCommand(0));


    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
