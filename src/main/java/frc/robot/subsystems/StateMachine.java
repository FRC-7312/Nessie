package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NessieState;
import frc.robot.PositionState;

public class StateMachine extends SubsystemBase {

  public static final NessieState STOW = new NessieState( // done
    "Stow", 14.21, 0, 0);
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
  public static final NessieState ALGAE_SHOOT = new NessieState(
    "Algae Shoot", 0, 5.2027880859375, 0);
  public static final NessieState ALGAE_INTAKE_LOW = new NessieState(
    "Algae Intake Low", 0, 0, 0);
  public static final NessieState ALGAE_INTAKE_HIGH = new NessieState(
    "Algae Intake High", 0, 1.41, 0);
  public static final NessieState ALGAE_PROCESS = new NessieState(
    "Algae Process", 0, 0, 0);
  public static final NessieState COAST = new NessieState(
    "COAST", -1, -1, -1);

  public static final PositionState LEFT_ALIGMENT_POSITION = new PositionState(
    -0.197, 0.253, -2.22);

  public static final PositionState RIGHT_ALIGMENT_POSITION = new PositionState(
    0.150, .262, .745);

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