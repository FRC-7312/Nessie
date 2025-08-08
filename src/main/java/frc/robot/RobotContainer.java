package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.LimelightHelpers;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.ChangeSpeedMultiplier;
import frc.robot.commands.ScoreLeft;
import frc.robot.commands.ScoreRight;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoIntake;
import frc.robot.commands.auto.AutoShootCoral;
import frc.robot.commands.endeffector.IntakeAndHoldAlgae;
import frc.robot.commands.endeffector.IntakeAndHoldCoral;
import frc.robot.commands.endeffector.ShootAlgae;
import frc.robot.commands.endeffector.ShootCoral;
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

    /* Commands */
    private final IntakeAndHoldCoral intakeAndHoldCoral;
    private final IntakeAndHoldAlgae intakeAndHoldAlgae;
    private final ShootAlgae shootAlgae;
    private final ShootCoral shootCoral;
    private final AutoShootCoral autoShootCoral;
    private final ChangeSpeedMultiplier changeSpeedMultiplier;
    private final ScoreLeft scoreLeft;
    private final ScoreRight scoreRight;
    private final AutoIntake autoIntake;

    /* Autos */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve,
                () -> -driver.getRawAxis(leftY), 
                () -> -driver.getRawAxis(leftX), 
                () -> -driver.getRawAxis(rightX), 
                () -> driver.getHID().getXButton(),
                () -> driver.getHID().getRightBumperButton()
            )
        );

        intakeAndHoldAlgae = new IntakeAndHoldAlgae(endEffector);
        intakeAndHoldAlgae.addRequirements(endEffector);
        intakeAndHoldCoral = new IntakeAndHoldCoral(endEffector);
        intakeAndHoldCoral.addRequirements(endEffector);
        shootAlgae = new ShootAlgae(endEffector);
        shootAlgae.addRequirements(endEffector);
        shootCoral = new ShootCoral(endEffector);
        shootCoral.addRequirements(endEffector);
        autoShootCoral = new AutoShootCoral(endEffector);
        autoShootCoral.addRequirements(endEffector);
        scoreLeft = new ScoreLeft(stateMachine, arm, elevator, swerve, endEffector);
        scoreRight = new ScoreRight(stateMachine, arm, elevator, swerve, endEffector);
        changeSpeedMultiplier = new ChangeSpeedMultiplier(swerve);
        changeSpeedMultiplier.addRequirements(swerve);
        autoIntake = new AutoIntake(stateMachine, endEffector);
        autoIntake.addRequirements(stateMachine, endEffector);

        /* register commands & autos */
        NamedCommands.registerCommand("Algae Intake Low Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_LOW)));
        NamedCommands.registerCommand("Algae Intake High Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_HIGH)));
        NamedCommands.registerCommand("Algae Shoot Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_SHOOT)));
        NamedCommands.registerCommand("Stow Position", new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW)));
        NamedCommands.registerCommand("Score Left", scoreLeft.onlyIf(() -> LimelightHelpers.getTV("limelight-left")));
        NamedCommands.registerCommand("Score Right", scoreRight.onlyIf(() -> LimelightHelpers.getTV("limelight-right")));
        NamedCommands.registerCommand("Intake", autoIntake);

        SmartDashboard.putData("Commands/Zero Elevator Encoder", new InstantCommand(() -> elevator.resetEncoder()));
        SmartDashboard.putData("Commands/Zero Arm Encoder", new InstantCommand(() -> arm.resetEncoder()));

        SmartDashboard.putData("Commands/Request Start State", new InstantCommand(() -> stateMachine.requestState(StateMachine.START)));
        SmartDashboard.putData("Commands/Request Disengaged State", new InstantCommand(() -> stateMachine.requestState(StateMachine.DISENGAGED)));
        SmartDashboard.putData("Commands/Request Stow State", new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW)));
        SmartDashboard.putData("Commands/Request L1 Sate", new InstantCommand(() -> stateMachine.requestState(StateMachine.L1)));
        SmartDashboard.putData("Commands/Request L2 Sate", new InstantCommand(() -> stateMachine.requestState(StateMachine.L2)));
        SmartDashboard.putData("Commands/Request L3 Sate", new InstantCommand(() -> stateMachine.requestState(StateMachine.L3)));
        SmartDashboard.putData("Commands/Request L4 Sate", new InstantCommand(() -> stateMachine.requestState(StateMachine.L4)));
        SmartDashboard.putData("Right Forward Position", new AlignToReefTagRelative("right forward", swerve));
        SmartDashboard.putData("Left Forward Position", new AlignToReefTagRelative("left forward", swerve));
        SmartDashboard.putData("Right Back Position", new AlignToReefTagRelative("right back", swerve));
        SmartDashboard.putData("Left Back Position", new AlignToReefTagRelative("left back", swerve));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);

        configureButtonBindings();

    }

    private void configureButtonBindings() {

        /* driver controls */

        driver.b().onTrue(changeSpeedMultiplier);
        driver.povUp().onTrue(new InstantCommand(() -> swerve.zeroHeading()));

        driver.leftTrigger().onTrue(
            new ScoreLeft(stateMachine, arm, elevator, swerve, endEffector)
                .onlyIf(() -> LimelightHelpers.getTV("limelight-left"))
        );
        driver.rightTrigger().onTrue(
            new ScoreRight(stateMachine, arm, elevator, swerve, endEffector)
                .onlyIf(() -> LimelightHelpers.getTV("limelight-right"))
        );
        driver.leftBumper().onTrue(
            new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW))
            .andThen(new WaitCommand(.2)).andThen(() -> stateMachine.requestState(StateMachine.INTAKE))
        );
        driver.leftBumper().whileTrue(intakeAndHoldCoral);
        driver.leftBumper().onFalse(
            new InstantCommand(() -> stateMachine.requestState(StateMachine.INTAKE_CLEARANCE))
            .andThen(new WaitCommand(.2)).andThen(() -> stateMachine.requestState(StateMachine.STOW))
        );

        /* operator controls */

        operator.a().onTrue(new InstantCommand(() -> stateMachine.setDesiredLevel(StateMachine.L1)));
        operator.b().onTrue(new InstantCommand(() -> stateMachine.setDesiredLevel(StateMachine.L2)));
        operator.y().onTrue(new InstantCommand(() -> stateMachine.setDesiredLevel(StateMachine.L3)));
        operator.x().onTrue(new InstantCommand(() -> stateMachine.setDesiredLevel(StateMachine.L4)));

        operator.povUp().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_SHOOT)));
        operator.povRight().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_HIGH)));
        operator.povLeft().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_INTAKE_LOW)));
        operator.povDown().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.ALGAE_PROCESS)));

        operator.leftBumper().whileTrue(intakeAndHoldAlgae);
        operator.rightBumper().whileTrue(shootAlgae);

        operator.leftTrigger().onTrue(new InstantCommand(() -> stateMachine.requestState(StateMachine.STOW)));
        operator.rightTrigger().whileTrue(shootCoral);

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
