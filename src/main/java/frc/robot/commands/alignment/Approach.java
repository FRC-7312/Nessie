package frc.robot.commands.alignment;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.PositionState;
import frc.robot.subsystems.Swerve;

public class Approach extends Command {

  private Swerve swerve;
  private PositionState targetPosition;

  PIDController yController;
  double tagID;

  public Approach(Swerve swerve, PositionState targetPosition) {
    this.targetPosition = targetPosition;
    this.swerve = swerve;

    yController = new PIDController(
      Constants.VisionConstants.Y_P,
      Constants.VisionConstants.Y_I,
      Constants.VisionConstants.Y_D
    );
    yController.setTolerance(Constants.VisionConstants.Y_TOLERANCE);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    tagID = LimelightHelpers.getFiducialID(targetPosition.getPerferredCameraName());
  }

  @Override
  public void execute() {
    double[] robotInTargetSpace = null;

    if (LimelightHelpers.getTV(Constants.VisionConstants.LEFT_LIMELIGHT_NAME) && 
        LimelightHelpers.getTV(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME) && 
        LimelightHelpers.getFiducialID(Constants.VisionConstants.LEFT_LIMELIGHT_NAME) == tagID && 
        LimelightHelpers.getFiducialID(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME) == tagID) {
      robotInTargetSpace = averageArrays(
        LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.LEFT_LIMELIGHT_NAME),
        LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME)
      );
      doDrive(robotInTargetSpace);
    }else if (LimelightHelpers.getTV(Constants.VisionConstants.LEFT_LIMELIGHT_NAME) && 
              LimelightHelpers.getFiducialID(Constants.VisionConstants.LEFT_LIMELIGHT_NAME) == tagID) {
      robotInTargetSpace = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.LEFT_LIMELIGHT_NAME);
      doDrive(robotInTargetSpace);
    }else if (LimelightHelpers.getTV(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME) && 
              LimelightHelpers.getFiducialID(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME) == tagID) {
      robotInTargetSpace = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME);
      doDrive(robotInTargetSpace);
    }else {
      swerve.drive(new Translation2d(), 0, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, true, true);
    System.out.println("Approach command ended" + (interrupted ? " due to interruption." : "."));
  }

  @Override
  public boolean isFinished() {
    return yController.atSetpoint();
  }

  private void doDrive(double[] robotInTargetSpace) {
    swerve.drive(
      new Translation2d(yController.calculate(robotInTargetSpace[1]), 0), 
      0, 
      false, 
      true
    );
  }

  public static double[] averageArrays(double[] a, double[] b) {
    double[] result = new double[a.length];
    for (int i = 0; i < a.length; i++) {
      result[i] = (a[i] + b[i]) / 2.0;
    }
    return result;
  }
}
