package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;
  private PositionVoltage positionVoltage;
  private NeutralOut neutralOut;
  private Canandmag canandmag;
  private CanandmagSettings canandmagSettings;
  private double desiredPosition;

  public Arm() {
    motor = new TalonFX(Constants.ID.ARM_PIVOT_ID);
    config = new TalonFXConfiguration();
    positionVoltage = new PositionVoltage(0).withSlot(0);
    neutralOut = new NeutralOut();
    canandmag = new Canandmag(Constants.ID.ARM_ENCODER_ID);
    canandmagSettings = new CanandmagSettings();
    canandmagSettings.setInvertDirection(true);
    canandmag.setPosition(0);

    config.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    config.Slot0.kI = 0; // No output for integrated error
    config.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    config.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    config.CurrentLimits.SupplyCurrentLimit = Constants.ArmConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ArmConstants.FORWARD_LIMIT*25;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ArmConstants.REVERSE_LIMIT*25;

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
    canandmag.setSettings(canandmagSettings);

    motor.setPosition(canandmag.getPosition()*25);
  }

  private double feedforward() {
    double halfPosition = 1.986328125; // this is what the position reads when the arm is straight up
    double kG = 0.014671875; // this is js the max feedforward, that theoeretically should be needed to hold the arm up in the .25 progression state

    double currentPosition = canandmag.getPosition();
    double progression = currentPosition/(halfPosition * 2);
    double progressionRads = progression * 2 *  Math.PI;
    double feedforward = kG * Math.sin(progressionRads);
    return feedforward;
  }

  public void setPosition(double desiredPosition) {
    motor.setControl(
      positionVoltage
        .withPosition(desiredPosition*25)
        .withFeedForward(feedforward())
    );
    this.desiredPosition = desiredPosition;
  }

  public boolean isFinished() {
    return MathUtil.isNear(desiredPosition, motor.getPosition().getValueAsDouble(), Constants.ArmConstants.TOLERANCE);
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
    SmartDashboard.putNumber("Arm/Setpoint", desiredPosition);
    SmartDashboard.putBoolean("Arm/Is Finished", isFinished());
    SmartDashboard.putNumber("Arm/Feedforward", feedforward());
    SmartDashboard.putNumber("Arm/Velocity", canandmag.getVelocity());
    SmartDashboard.putNumber("Arm/Position", Double.parseDouble(String.format("%.2f", canandmag.getPosition())));
    SmartDashboard.putNumber("Arm/Motor/Velocity", motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Arm/Motor/Applied Output", motor.get());
    SmartDashboard.putNumber("Arm/Motor/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Arm/Motor/Position", motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm/Motor/Output Voltage", motor.getMotorVoltage().getValueAsDouble());
  }

}