package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;
  private double desiredPosition;

  public Arm() {
    motor = new TalonFX(Constants.ID.ARM_PIVOT_ID);
    config = new TalonFXConfiguration();

    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.CurrentLimits.SupplyCurrentLimit = Constants.ArmConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ArmConstants.FORWARD_LIMIT;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ArmConstants.REVERSE_LIMIT;

    motor.getConfigurator().apply(config);

    motor.setControl(new CoastOut());

    SmartDashboard.putData("Arm/Set Coast Mode", setCoastCommand());
    SmartDashboard.putData("Arm/Reset Position", resetPositionCommand());

    System.out.println("Arm subsystem initialized");
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
    System.out.println("Arm set to position: " + desiredPosition);
  }

  public Command setCoastCommand() {
    return runOnce(() -> {
      setCoast();
    });
  }

  public void setCoast() {
    motor.setControl(new CoastOut());
    System.out.println("Arm set to coast mode");
  }

  public Command resetPositionCommand() {
    return runOnce(() -> {
      resetPosition();
    });
  }

  public void resetPosition() {
    motor.setPosition(0);
    System.out.println("Arm position reset to 0");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Setpoint", desiredPosition);
    SmartDashboard.putNumber("Arm/Position", Double.parseDouble(String.format("%.2f", motor.getPosition().getValueAsDouble())));
  }

}