package frc.robot;

import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.LimelightHelpers;
import frc.robot.subsystems.RevBlinkin;

public class Robot extends TimedRobot {

  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private Command auto;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    CanandEventLoop.getInstance();
    robotContainer = new RobotContainer();
    RevBlinkin.getInstance().setPattern(RevBlinkin.BlinkinPattern.RED);
  }

  @Override
  public void robotPeriodic() {       
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());

    updateVisionTelemetry();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    auto = robotContainer.getAutonomousCommand();

    if (auto != null) {
      auto.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (auto != null) {
      auto.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  private void updateVisionTelemetry() {

    double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
    if(positions.length > 0) {
      SmartDashboard.putNumber("Vision/Left/X", positions[0]);
      SmartDashboard.putNumber("Vision/Left/Y", positions[1]);
      SmartDashboard.putNumber("Vision/Left/Yaw", positions[4]);
    }

    positions = LimelightHelpers.getBotPose_TargetSpace("limelight-right");
    if(positions.length > 0) {
      SmartDashboard.putNumber("Vision/Right/X", positions[0]);
      SmartDashboard.putNumber("Vision/Right/Y", positions[1]);
      SmartDashboard.putNumber("Vision/Right/Yaw", positions[4]);
    }

  }
}