package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;;

public class Elevator extends SubsystemBase {

  private TalonFX motor;
  private TalonFX follower;
  private TalonFXConfiguration config;
  private PositionVoltage positionVoltage;
  private NeutralOut neutralOut;
  private Canandmag canandmag;
  private CanandmagSettings canandmagSettings;
  private double desiredPosition;

  public Elevator() {
    motor = new TalonFX(Constants.ID.ELEVATOR_MASTER_ID);
    follower = new TalonFX(Constants.ID.ELEVATOR_FOLLOWER_ID);
    config = new TalonFXConfiguration();
    positionVoltage = new PositionVoltage(0).withSlot(0);
    neutralOut = new NeutralOut();
    canandmag = new Canandmag(Constants.ID.ELEVATOR_ENCODER_ID);
    canandmagSettings = new CanandmagSettings();
    canandmagSettings.setInvertDirection(true);
    canandmag.setPosition(0);

    config.Slot0.kP = 1.2; // An error of 1 rotation results in 2.4 V output
    config.Slot0.kI = 0; // No output for integrated error
    config.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    config.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    config.CurrentLimits.SupplyCurrentLimit = Constants.ElevatorConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
 
    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
    canandmag.setSettings(canandmagSettings);

    follower.setControl(new Follower(motor.getDeviceID(), true));

    motor.setPosition(canandmag.getPosition()*9);
  }

  public void setPosition(double desiredPosition) {
    motor.setControl(
      positionVoltage
        .withPosition(desiredPosition*9)
    );
    this.desiredPosition = desiredPosition;
  }

  public boolean isFinished() {
    return MathUtil.isNear(desiredPosition, motor.getPosition().getValueAsDouble(), Constants.ElevatorConstants.TOLERANCE);
  }

  public void resetEncoder() {
    canandmag.setPosition(0);
    motor.setPosition(canandmag.getPosition());
  }

  public void disengage() {
    motor.setControl(neutralOut);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/Setpoint", desiredPosition);
    SmartDashboard.putBoolean("Elevator/Is Finished", isFinished());
    SmartDashboard.putNumber("Elevator/Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Elevator/Position", Double.parseDouble(String.format("%.2f", canandmag.getPosition())));
    SmartDashboard.putNumber("Elevator/Motor/Velocity", motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Motor/Applied Output", motor.get());
    SmartDashboard.putNumber("Elevator/Motor/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Motor/Position", motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Motor/Output Voltage", motor.getMotorVoltage().getValueAsDouble());
  }

}