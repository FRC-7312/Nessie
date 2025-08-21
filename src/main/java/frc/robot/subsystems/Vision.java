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
      0.1817,    // Forward offset (meters)
      -0.0499,    // Side offset (meters)
      0.1859,    // Height offset (meters)
      -22.500000,    // Roll (degrees)
      14.432786,   // Pitch (degrees)
      0.0     // Yaw (degrees)
    );

    LimelightHelpers.setCameraPose_RobotSpace(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME, 
      0.1817,    // Forward offset (meters)
      0.0499,    // Side offset (meters)
      0.1859,    // Height offset (meters)
      22.500000,    // Roll (degrees)
      14.432786,   // Pitch (degrees)
      0.0     // Yaw (degrees)
    );

  }

  @Override
  public void periodic() { // [x, y, z, roll, pitch, yaw]
    double[] leftBotPose = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.LEFT_LIMELIGHT_NAME);
    double[] rightBotPose = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME);
    SmartDashboard.putNumber("Vision/Closest Valid Target ID", LimelightHelpers.getClosestFiducial(Constants.VisionConstants.LEFT_LIMELIGHT_NAME, Constants.VisionConstants.RIGHT_LIMELIGHT_NAME).fiducialID);
    if(leftBotPose != null && rightBotPose != null) {
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Average/X", (leftBotPose[0] + rightBotPose[0]) / 2.0);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Average/Y", (leftBotPose[1] + rightBotPose[1]) / 2.0);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Average/Z", (leftBotPose[2] + rightBotPose[2]) / 2.0);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Average/Roll", (leftBotPose[3] + rightBotPose[3]) / 2.0);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Average/Pitch", (leftBotPose[4] + rightBotPose[4]) / 2.0);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Average/Yaw", (leftBotPose[5] + rightBotPose[5]) / 2.0);
    }
    if(leftBotPose != null) {
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Left/X", leftBotPose[0]);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Left/Y", leftBotPose[1]);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Left/Z", leftBotPose[2]);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Left/Roll", leftBotPose[3]);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Left/Pitch", leftBotPose[4]);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Left/Yaw", leftBotPose[5]);
    }
    if(rightBotPose != null) {
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Right/X", rightBotPose[0]);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Right/Y", rightBotPose[1]);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Right/Z", rightBotPose[2]);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Right/Roll", rightBotPose[3]);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Right/Pitch", rightBotPose[4]);
      SmartDashboard.putNumber("Vision/Bot Pose Target Space/Right/Yaw", rightBotPose[5]);
    }
  }
}

/*
 * VISION TODO
 * 
 * - calibrate cameras
 * - set camera pipeline
 * - test cameras locally
 * 
 */