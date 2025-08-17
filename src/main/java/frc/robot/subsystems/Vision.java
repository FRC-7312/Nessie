package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.PositionState;

public class Vision extends SubsystemBase {

  public static final PositionState Base = new PositionState(0.0, 1, 0.0);

  public static final PositionState Left = new PositionState(0.0, 1, 0.0);
  public static final PositionState Right = new PositionState(0.0, 1, 0.0);

  public Vision(Swerve swerve) {

    LimelightHelpers.setCameraPose_RobotSpace(Constants.VisionConstants.LEFT_LIMELIGHT_NAME, 
      0.5,    // Forward offset (meters)
      0.0,    // Side offset (meters)
      0.5,    // Height offset (meters)
      0.0,    // Roll (degrees)
      30.0,   // Pitch (degrees)
      0.0     // Yaw (degrees)
    );

    LimelightHelpers.setCameraPose_RobotSpace(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME, 
      0.5,    // Forward offset (meters)
      0.0,    // Side offset (meters)
      0.5,    // Height offset (meters)
      0.0,    // Roll (degrees)
      30.0,   // Pitch (degrees)
      0.0     // Yaw (degrees)
    );

  }

  @Override
  public void periodic() { // [x, y, z, roll, pitch, yaw]
    double[] leftBotPose = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.LEFT_LIMELIGHT_NAME);
    double[] rightBotPose = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Left/x", leftBotPose[0]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Left/y", leftBotPose[1]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Left/z", leftBotPose[2]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Left/roll", leftBotPose[3]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Left/pitch", leftBotPose[4]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Left/yaw", leftBotPose[5]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Right/x", rightBotPose[0]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Right/y", rightBotPose[1]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Right/z", rightBotPose[2]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Right/roll", rightBotPose[3]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Right/pitch", rightBotPose[4]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Right/yaw", rightBotPose[5]);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Average/x", (leftBotPose[0] + rightBotPose[0]) / 2.0);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Average/y", (leftBotPose[1] + rightBotPose[1]) / 2.0);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Average/z", (leftBotPose[2] + rightBotPose[2]) / 2.0);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Average/roll", (leftBotPose[3] + rightBotPose[3]) / 2.0);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Average/pitch", (leftBotPose[4] + rightBotPose[4]) / 2.0);
    SmartDashboard.putNumber("Vision/BotPoseTargetSpace/Average/yaw", (leftBotPose[5] + rightBotPose[5]) / 2.0);
  }

}
