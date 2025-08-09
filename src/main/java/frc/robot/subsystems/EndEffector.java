package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration config;

  public EndEffector() {
    motor = new TalonFX(Constants.ID.ENDEFFECTOR_ID);
    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = Constants.EndEffectorConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);

    motor.setControl(new NeutralOut());

    SmartDashboard.putData("Endeffector/Set Brake", setBrakeCommand());

    System.out.println("EndEffector subsystem initialized");
  }

  public Command setVoltageCommand(double desiredVoltage) {
    return runOnce(() -> {
      setVoltage(desiredVoltage);
    });
  }

  public void setVoltage(double desiredVoltage) {
    motor.setControl(
      new VoltageOut(desiredVoltage)
    );
    System.out.println("EndEffector voltage set to: " + desiredVoltage);
  }

  public Command setBrakeCommand() {
    return runOnce(() -> {
      setBrake();
    });
  }

  public void setBrake() {
    motor.setControl(new NeutralOut());
    System.out.println("EndEffector set to brake mode");
  }

  @Override
  public void periodic() {}
  
}