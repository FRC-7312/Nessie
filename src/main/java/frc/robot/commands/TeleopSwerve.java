package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier lockSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier lockSup, BooleanSupplier robotCentricSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.lockSup = lockSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.ControlConstants.STICK_DEADBAND);

        if(!lockSup.getAsBoolean()) {
            swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed), 
                rotationVal * Constants.SwerveConstants.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
        }else {
            swerve.setX();
        }
    }
}