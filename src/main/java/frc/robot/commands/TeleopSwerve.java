package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.LimelightReader;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
// debug utility
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.xLimiter = new SlewRateLimiter(Constants.Swerve.maxSpeed);
        this.yLimiter = new SlewRateLimiter(Constants.Swerve.maxSpeed);
        this.turningLimiter = new SlewRateLimiter(Constants.Swerve.maxAngularVelocity);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        // double rotationVal = 0;

        // translationVal = xLimiter.calculate(translationVal) * Constants.Swerve.maxSpeed;
        // strafeVal = yLimiter.calculate(strafeVal) * Constants.Swerve.maxSpeed;
        // rotationVal = turningLimiter.calculate(rotationVal)
        // * Constants.Swerve.maxAngularVelocity;

        // double translationVal = translationSup.getAsDouble();
        // double strafeVal = strafeSup.getAsDouble();
        // double rotationVal = 0;

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            // !robotCentricSup.getAsBoolean(), 
            false,
            false
        );
        //debug
        SmartDashboard.putNumber("translationVal", translationVal);
        SmartDashboard.putNumber("strafeVal", strafeVal);
        SmartDashboard.putNumber("rotationVal", rotationVal);
    }
}