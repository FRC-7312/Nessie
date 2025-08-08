package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NessieState;
import frc.robot.commands.position.SetArmPosition;
import frc.robot.commands.position.SetElevatorPosition;

public class StateMachine extends SubsystemBase {

  public static final NessieState START = new NessieState(
    "Start", 0, 0, 0);
  public static final NessieState STOW = new NessieState( // done
    "Stow", .5, 0, 0);
  public static final NessieState L1 = new NessieState(
    "L1", .3, 0, .08);
  public static final NessieState L2 = new NessieState( // done
    "L2", 1.37799072265625, 0, 0);
  public static final NessieState L3 = new NessieState( // done
    "L3", 1.72351318359375, 0.86962890625, 0);
  public static final NessieState L4 = new NessieState( // done
    "L4", 1.70, 5.71, 0);
  public static final NessieState INTAKE = new NessieState(
    "Intake", 0.12, 0, 0);
  public static final NessieState INTAKE_CLEARANCE = new NessieState(
    "Intake Clearance", .12, 1.86, 0);
  public static final NessieState ALGAE_TAXI= new NessieState(
    "Algae Stow", 1.93, 0, 0);
  public static final NessieState ALGAE_SHOOT = new NessieState(
    "Algae Shoot", 1.7, 5.2027880859375, 0);
  public static final NessieState ALGAE_INTAKE_LOW = new NessieState(
    "Algae Intake Low", .83, 0, 0);
  public static final NessieState ALGAE_INTAKE_HIGH = new NessieState(
    "Algae Intake High", 1.01, 1.41, 0);
  public static final NessieState ALGAE_PROCESS = new NessieState(
    "Algae Process", 0, 0, 0);
  public static final NessieState DISENGAGED = new NessieState(
    "Disengaged", -1, -1, -1);

  private NessieState currentState;
  public NessieState desiredLevel = L4;

  private Elevator elevator;
  private Arm arm;

  public StateMachine(Elevator elevator, Arm arm) {
    this.elevator = elevator;
    this.arm = arm;

    currentState = DISENGAGED;
  }

  public void setDesiredLevel(NessieState desiredLevel) {
    this.desiredLevel = desiredLevel;
  }

  public NessieState getDesiredLevel() {
    return desiredLevel;
  }

  public void requestState(NessieState desiredState) {
    if(desiredState == DISENGAGED) {
      new InstantCommand(() -> arm.disengage()).schedule();
      new InstantCommand(() -> elevator.disengage()).schedule();
    }else {
      new SetArmPosition(arm, desiredState.getArmPosition()).schedule();
      new SetElevatorPosition(elevator, desiredState.getElevatorPosition()).schedule();
    }
    
    currentState = desiredState;
  }

  public boolean stateReached() {
    return elevator.isFinished() && arm.isFinished();
  }

  public NessieState getCurrentState() {
    return currentState;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("StateMachine/Current State", currentState.getName());
    SmartDashboard.putBoolean("StateMachine/State Reached", stateReached());
    SmartDashboard.putString("StateMachine/Desired Level", desiredLevel.getName());
  }

}