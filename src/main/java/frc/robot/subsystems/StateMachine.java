package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.NessieState;
import frc.robot.PositionState;

public class StateMachine extends SubsystemBase {

  public static final NessieState STOW = new NessieState( // done
    "Stow", 10.9, 0, 0);
  public static final NessieState L1 = new NessieState(
    "L1", 0, 0, 0);
  public static final NessieState L2 = new NessieState( // done
    "L2", 35.43, 0, 0);
  public static final NessieState L3 = new NessieState( // done
    "L3", 45.58, 0, 0);
  public static final NessieState L4 = new NessieState( // done
    "L4", 42.7, 50.40, 0);
  public static final NessieState INTAKE = new NessieState(
    "Intake", 2.07, 0, 0);
  public static final NessieState INTAKE_CLEARANCE = new NessieState(
    "Intake Clearance", 0, 1.86, 0);
  public static final NessieState ALGAE_TAXI= new NessieState(
    "Algae Stow", 49.15, 0, 0);
  public static final NessieState ALGAE_PRE_YEET = new NessieState(
    "Algae Pre-Yeet", 50.56, 50.91, 0);
    public static final NessieState ALGAE_YEET = new NessieState(
    "Algae Yeet", 36.71, 51.49, 0);
  public static final NessieState ALGAE_INTAKE_LOW = new NessieState(
    "Algae Intake Low", 19.06, 0, 0);
  public static final NessieState ALGAE_INTAKE_HIGH = new NessieState(
    "Algae Intake High", 23.79, 12.56, 0);
  public static final NessieState ALGAE_PROCESS = new NessieState(
    "Algae Process", 0, 0, 0);
  public static final NessieState ALGAE_LICK = new NessieState(
    "Algae Lick", 10.7, 0, 0);
  public static final NessieState COAST = new NessieState(
    "COAST", -1, -1, -1);

  public static final PositionState LEFT_FRONT = new PositionState(
    -0.197, 0.253, -2.22, Constants.VisionConstants.LEFT_LIMELIGHT_NAME);

  public static final PositionState RIGHT_FORWARD = new PositionState(
    0.150, .262, .745, Constants.VisionConstants.RIGHT_LIMELIGHT_NAME);

  public static final PositionState LEFT_BACK = new PositionState(
    0, 0, 0, Constants.VisionConstants.LEFT_LIMELIGHT_NAME); // tune

  public static final PositionState RIGHT_BACK = new PositionState(
    0, 0, 0, Constants.VisionConstants.RIGHT_LIMELIGHT_NAME); // tune

  private NessieState currentState;

  private Elevator elevator;
  private Arm arm;

  public StateMachine(Elevator elevator, Arm arm) {
    this.elevator = elevator;
    this.arm = arm;

    currentState = COAST;

    SmartDashboard.putString("StateMachine/Current State", currentState.getName());

    System.out.println("StateMachine subsystem initialized");
  }

  public void requestState(NessieState desiredState) {
    if(desiredState == COAST) {
      arm.setCoastCommand().schedule();
      elevator.setCoastCommand().schedule();
    }else {
      arm.setPositionCommand(desiredState.getArmPosition()).schedule();
      elevator.setPositionCommand(desiredState.getElevatorPosition()).schedule();
    }
    currentState = desiredState;

    SmartDashboard.putString("StateMachine/Current State", currentState.getName());

    System.out.println("StateMachine requested state: " + desiredState.getName());
  }

  public Command requestStateCommand(NessieState desiredState) {
    return runOnce(() -> requestState(desiredState));
  }

  @Override
  public void periodic() {}

}