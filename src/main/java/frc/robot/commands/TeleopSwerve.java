package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.PositionState;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;


public class TeleopSwerve extends Command {    
    
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier alignLeftSupplier;
    private BooleanSupplier alignRightSupplier;
    private BooleanSupplier lockSup;

    private PIDController xController = new PIDController(
        Constants.VisionConstants.translationPID.kP, 
        Constants.VisionConstants.translationPID.kI, 
        Constants.VisionConstants.translationPID.kD
    );

    private PIDController yController = new PIDController(
        Constants.VisionConstants.translationPID.kP, 
        Constants.VisionConstants.translationPID.kI, 
        Constants.VisionConstants.translationPID.kD
    );

    private PIDController rotationController = new PIDController(
        Constants.VisionConstants.rotationPID.kP, 
        Constants.VisionConstants.rotationPID.kI, 
        Constants.VisionConstants.rotationPID.kD
    );

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
    public void execute() {
        double[] leftBotPose = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.LEFT_LIMELIGHT_NAME);
        double[] rightBotPose = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME);

        boolean didSomething = false;

        // Priority 2: Left/Right lock alignment
        if (alignLeftSupplier.getAsBoolean() || alignRightSupplier.getAsBoolean()) {

            System.out.println("Align Suppliers Triggered");

            double x = -999, y = -999, yaw = -999;

            if (LimelightHelpers.getTV(Constants.VisionConstants.LEFT_LIMELIGHT_NAME) && LimelightHelpers.getTV(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME)) {
                x = (leftBotPose[0] + rightBotPose[0]) / 2.0;
                y = (leftBotPose[2] + rightBotPose[2]) / 2.0;
                yaw = (leftBotPose[4] + rightBotPose[4]) / 2.0;
                SmartDashboard.putNumber("Vision/Average/X", (leftBotPose[0] + rightBotPose[0]) / 2.0);
                SmartDashboard.putNumber("Vision/Average/Y", (leftBotPose[2] + rightBotPose[2]) / 2.0);
                SmartDashboard.putNumber("Vision/Average/Yaw", (leftBotPose[4] + rightBotPose[4]) / 2.0);
            } else if (LimelightHelpers.getTV(Constants.VisionConstants.LEFT_LIMELIGHT_NAME)) {
                x = leftBotPose[0];
                y = leftBotPose[2];
                yaw = leftBotPose[4];
                SmartDashboard.putNumber("Vision/Left/X", leftBotPose[0]);
                SmartDashboard.putNumber("Vision/Left/Y", leftBotPose[2]);
                SmartDashboard.putNumber("Vision/Left/Yaw", leftBotPose[4]);
            } else if (LimelightHelpers.getTV(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME)) {
                x = rightBotPose[0];
                y = rightBotPose[2];
                yaw = rightBotPose[4];
                SmartDashboard.putNumber("Vision/Right/X", rightBotPose[0]);
                SmartDashboard.putNumber("Vision/Right/Y", rightBotPose[2]);
                SmartDashboard.putNumber("Vision/Right/Yaw", rightBotPose[4]);
            } else {
                return;
            }

            PositionState targetPosition = alignLeftSupplier.getAsBoolean() 
                ? StateMachine.LEFT_FRONT 
                : StateMachine.RIGHT_FORWARD;

            SmartDashboard.putNumber("Vision/Alignment Setpoint/Current X Measurement", x);
            SmartDashboard.putNumber("Vision/Alignment Setpoint/Current Y Measurement", y);
            SmartDashboard.putNumber("Vision/Alignment Setpoint/Current Yaw Measurement", yaw);
            SmartDashboard.putNumber("Vision/Alignment Setpoint/Target X", targetPosition.getX());
            SmartDashboard.putNumber("Vision/Alignment Setpoint/Target Y", targetPosition.getY());
            SmartDashboard.putNumber("Vision/Alignment Setpoint/Target Yaw", targetPosition.getYaw());

            swerve.drive(
                new Translation2d(MathUtil.applyDeadband(
                    translationSup.getAsDouble(), 
                    Constants.ControlConstants.STICK_DEADBAND) , -yController.calculate(x, targetPosition.getX())), 
                -rotationController.calculate(yaw, targetPosition.getYaw()), 
                false, 
                true
            );
            didSomething = true;

        }

        // Priority 2: X lock
        if (!didSomething && lockSup.getAsBoolean()) {
            swerve.setX();
            didSomething = true;
        }

        // Priority 3: Normal drive
        if (!didSomething) {
            double translationVal = MathUtil.applyDeadband(
                translationSup.getAsDouble(), 
                Constants.ControlConstants.STICK_DEADBAND
            );
            double strafeVal = MathUtil.applyDeadband(
                strafeSup.getAsDouble(), 
                Constants.ControlConstants.STICK_DEADBAND
            );
            double rotationVal = MathUtil.applyDeadband(
                rotationSup.getAsDouble(), 
                Constants.ControlConstants.STICK_DEADBAND
            );

            swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed), 
                rotationVal * Constants.SwerveConstants.maxAngularVelocity, 
                true, 
                true
            );
        }
    }

}