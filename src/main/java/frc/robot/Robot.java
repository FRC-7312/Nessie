package frc.robot;

import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.Elastic;
import frc.lib.Elastic.Notification;
import frc.lib.LimelightHelpers;
import frc.robot.subsystems.RevBlinkin;

public class Robot extends TimedRobot {

  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private Command auto;
  private RobotContainer robotContainer;
  private double batteryLowStartTime = -1;
  public static RevBlinkin revBlinkin = new RevBlinkin();

  @Override
  public void robotInit() {
    CanandEventLoop.getInstance();
    robotContainer = new RobotContainer();
    revBlinkin.redSolidColor();
    SmartDashboard.putBoolean("Alignment/Valid Tag", false);
  }

  @Override
  public void robotPeriodic() {       
    CommandScheduler.getInstance().run();
    if (RobotController.getBatteryVoltage() < Constants.NotificationConstants.BATTERY_LOW_VOLTAGE) {
      if (batteryLowStartTime == -1) {
          batteryLowStartTime = Timer.getFPGATimestamp();
      }
      if (Timer.getFPGATimestamp() - batteryLowStartTime >= Constants.NotificationConstants.BATTERY_LOW_DURATION) {
          Elastic.sendNotification(new Notification(
              Elastic.Notification.NotificationLevel.WARNING,
              "Battery Voltage Low",
              "Battery Voltage is below " + 
              Constants.NotificationConstants.BATTERY_LOW_VOLTAGE + 
              "V for " + Constants.NotificationConstants.BATTERY_LOW_DURATION + " seconds"));
          batteryLowStartTime = -1;
      }
    }
    else {
        batteryLowStartTime = -1;
    }

    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    
    double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
    if(positions.length > 0) {
      SmartDashboard.putNumber("Alignment/Left/X", positions[0]);
      SmartDashboard.putNumber("Alignment/Left/Y", positions[1]);
      SmartDashboard.putNumber("Alignment/Left/Yaw", positions[4]);
      SmartDashboard.putBoolean("Alignment/Left/Valid Tag", LimelightHelpers.getTV("limelight-left"));
    }

    positions = LimelightHelpers.getBotPose_TargetSpace("limelight-right");
    if(positions.length > 0) {
      SmartDashboard.putNumber("Alignment/Right/X", positions[0]);
      SmartDashboard.putNumber("Alignment/Right/Y", positions[1]);
      SmartDashboard.putNumber("Alignment/Right/Yaw", positions[4]);
      SmartDashboard.putBoolean("Alignment/Right/Valid Tag", LimelightHelpers.getTV("limelight-right"));
    }

    if(LimelightHelpers.getTV("limelight-right") || LimelightHelpers.getTV("limelight-right")) {
      revBlinkin.strobeRedPattern();
    }else {
      revBlinkin.redSolidColor();
    }

  }

  @Override
  public void disabledInit() {
    revBlinkin.redSolidColor();
  }

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
    revBlinkin.redSolidColor();
    if (auto != null) {
      auto.cancel();
    }
    new InstantCommand().schedule();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}