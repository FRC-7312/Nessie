package frc.robot.commands;

import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier alignLeftSupplier;
    private BooleanSupplier alignRightSupplier;
    private BooleanSupplier lockSup;

    public TeleopSwerve(Swerve swerve, 
                        DoubleSupplier translationSup, 
                        DoubleSupplier strafeSup, 
                        DoubleSupplier rotationSup, 
                        BooleanSupplier alignLeftSupplier,
                        BooleanSupplier alignRightSupplier,
                        BooleanSupplier lockSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.alignLeftSupplier = alignLeftSupplier;
        this.alignRightSupplier = alignRightSupplier;
        this.lockSup = lockSup;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);
        double closestTagID;

        if(alignLeftSupplier.getAsBoolean() || alignRightSupplier.getAsBoolean()) {
            LimelightTarget_Fiducial closestTag = 
                LimelightHelpers.getClosestFiducial(Constants.VisionConstants.LEFT_LIMELIGHT_NAME, Constants.VisionConstants.RIGHT_LIMELIGHT_NAME);
            if (closestTag != null) { // has valid tag
                PPHolonomicDriveController controller = new PPHolonomicDriveController(
                    Constants.VisionConstants.translationPID, 
                    Constants.VisionConstants.rotationPID
                );
                closestTagID = closestTag.fiducialID;
                double leftID = LimelightHelpers.getFiducialID(Constants.VisionConstants.LEFT_LIMELIGHT_NAME);
                double rightID = LimelightHelpers.getFiducialID(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME);
                double[] leftBotPose = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.LEFT_LIMELIGHT_NAME);
                double[] rightBotPose = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME);
                double x = -1, y = -1, yaw = -1;

                if(leftID == closestTagID && rightID == closestTagID) {
                    x = (leftBotPose[0] + rightBotPose[0]) / 2.0;
                    y = (leftBotPose[1] + rightBotPose[1]) / 2.0;
                    yaw = (leftBotPose[5] + rightBotPose[5]) / 2.0;
                }else if(leftID == closestTagID) {
                    x = leftBotPose[0];
                    y = leftBotPose[1];
                    yaw = leftBotPose[5];
                }else if(rightID == closestTagID) {
                    x = rightBotPose[0];
                    y = rightBotPose[1];
                    yaw = rightBotPose[5];
                }

                PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
                targetState.pose = alignLeftSupplier.getAsBoolean() ? 
                    new Pose2d(new Translation2d(Vision.Left.getX(), Vision.Left.getY()), new Rotation2d(Units.degreesToRadians(Vision.Left.getYaw()))) : 
                    new Pose2d(new Translation2d(Vision.Right.getX(), Vision.Right.getY()), new Rotation2d(Units.degreesToRadians(Vision.Right.getYaw())));

                ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(
                    new Pose2d(new Translation2d(x, y), new Rotation2d(yaw)), 
                    targetState
                );

                swerve.driveRobotRelative(speeds);
            }
        }else if(lockSup.getAsBoolean()) {
            swerve.setX();
        }else {
            swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed), 
                rotationVal * Constants.SwerveConstants.maxAngularVelocity, 
                true, 
                true
            );
        }
    }
}