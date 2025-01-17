package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.LimelightReader;
import frc.robot.subsystems.LimelightReader.RetDistance;
import frc.robot.subsystems.Swerve;

public class ApriltagAutoAlign extends Command {
    private final Swerve m_swerve;
    private final DoublePublisher ctrlVal, errVal;
    private final LimelightReader reader;
    private final double kP, kI, kD, tP, tE;
    private final PIDController pidController;
    private Double errorX, setVal;
    private RetDistance distance;

    /**
     * 
     * 
     * @param Swerve readily allocated swerve object
     */
    public ApriltagAutoAlign(Swerve swerve) {
        /**
         * No PID implementation for now
         */
        kP = 0.5;
        kI = 0;
        kD = 0;
        // position tolerance and error tolerance
        tP = 0.05;
        tE = 0.23;
        NetworkTable table = NetworkTableInstance.getDefault().getTable("ciallo");
        ctrlVal = table.getDoubleTopic("ctrlVal").publish();
        errVal = table.getDoubleTopic("errVal").publish();
        this.pidController = new PIDController(kP, kI, kD);
        this.m_swerve = swerve;
        addRequirements(m_swerve);
        this.reader = new LimelightReader();
        // this.distance = new LimelightReader().getDistance();
    }

    @Override
    public void initialize() {
        this.distance = new LimelightReader().getDistance();
        pidController.setTolerance(tP, tE);
    }

    @Override
    public void execute() {
        distance = reader.getDistance();
        // SmartDashboard.putNumber("testInstance", distance.getX());
        // m_swerve.drive(new Translation2d(0.0,
        // distance.getX()).times(Constants.Swerve.maxSpeed), 0, false, true);
        errorX = distance.getX();
        errorX = (errorX.isNaN() ? 0.0 : errorX);
        if (errorX == 0.0) {
            
        }
        setVal = pidController.calculate(distance.getX());
        ctrlVal.set(setVal);
        errVal.set(distance.getX());
        m_swerve.drive(new Translation2d(0.0,
                setVal).times(Constants.Swerve.maxSpeed), 0, false, true);
    }

}
