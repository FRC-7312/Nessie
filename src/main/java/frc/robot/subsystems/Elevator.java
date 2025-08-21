package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;;

public class Elevator extends SubsystemBase {

  private TalonFX motor;
  private TalonFX follower;
  private TalonFXConfiguration config;
  private double desiredPosition;

  public Elevator() {
    motor = new TalonFX(Constants.ID.ELEVATOR_MASTER_ID);
    follower = new TalonFX(Constants.ID.ELEVATOR_FOLLOWER_ID);
    config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.ElevatorConstants.P;
    config.Slot0.kI = Constants.ElevatorConstants.I;
    config.Slot0.kD = Constants.ElevatorConstants.D;

    config.CurrentLimits.SupplyCurrentLimit = Constants.ElevatorConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ElevatorConstants.FORWARD_LIMIT;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ElevatorConstants.REVERSE_LIMIT;
 
    motor.getConfigurator().apply(config);

    follower.setControl(new Follower(motor.getDeviceID(), true));

    motor.setControl(new CoastOut());

    SmartDashboard.putData("Elevator/Set Coast Mode", setCoastCommand());
    SmartDashboard.putData("Elevator/Reset Position", resetPositionCommand());

    System.out.println("Elevator subsystem initialized");
  }

  public Command setPositionCommand(double desiredPosition) {
    return runOnce(() -> {
      setPosition(desiredPosition);
    });
  }

  public void setPosition(double desiredPosition) {
    this.desiredPosition = desiredPosition;
    motor.setControl(
      new PositionVoltage(desiredPosition)
        .withSlot(0)
    );
    System.out.println("Elevator set to position: " + desiredPosition);
  }

  public Command resetPositionCommand() {
    return runOnce(() -> {
      resetPosition();
    });
  }

  public void resetPosition() {
    motor.setPosition(0);
    System.out.println("Elevator position reset");
  }


  public Command setCoastCommand() {
    return runOnce(() -> {
      setCoast();
    });
  }

  public void setCoast() {
    motor.setControl(new CoastOut());
    System.out.println("Elevator set to coast mode");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/Setpoint", desiredPosition);
    SmartDashboard.putNumber("Elevator/Position", Double.parseDouble(String.format("%.2f", motor.getPosition().getValueAsDouble())));
  }

}