package frc.robot.subsystems.drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.constants.Constants.DriveConstants;

public class SwerveModule {

    public SparkMax driveMotor;
    public SparkMax steerMotor;
    public RelativeEncoder driveNEOMotorEncoder; // NEO build-in Encoder

    public CANcoder steerAngleEncoder;

    private PIDController steerAnglePID;
    private SparkClosedLoopController steerMotorVelocityPID;
    private SparkClosedLoopController driveMotorVelocityPID;

    public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderId, boolean invertDrive) {

        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);

        steerAngleEncoder = new CANcoder(steerEncoderId);

        driveNEOMotorEncoder = driveMotor.getEncoder();

        /// PID Controllers ///
        steerAnglePID = new PIDController(DriveConstants.PID_Encoder_Steer.p, DriveConstants.PID_Encoder_Steer.i, DriveConstants.PID_Encoder_Steer.d);
        steerAnglePID.enableContinuousInput(-180, 180);

        // Set the motor controller PIDs
        DriveConstants.PID_SparkMax_Drive.setSparkMaxPID(driveMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        DriveConstants.PID_SparkMax_Steer.setSparkMaxPID(steerMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMotorVelocityPID = steerMotor.getClosedLoopController();
        driveMotorVelocityPID = driveMotor.getClosedLoopController();

        // This must come after the PID setting because kResetSafeParameters resets all values to safe defaults before setting anything.
        // invertDrive is per-module because the left and right sides of the chassis are physical
        // mirror images of each other -- one hardcoded inversion can only ever be correct for one side.
        SparkMaxConfig driveEncoderConfig = new SparkMaxConfig();
        driveEncoderConfig
            .inverted(invertDrive)
            .encoder.positionConversionFactor(DriveConstants.DRIVE_GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE);
        driveMotor.configure(driveEncoderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Returns the current state of the module.
     * 
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double driveSpeed = speedFromDriveRpm(driveNEOMotorEncoder.getVelocity());
        double steerAngleRadians = Math.toRadians(steerAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360);

        return new SwerveModuleState(driveSpeed, new Rotation2d(steerAngleRadians));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean logValues, String name) {

        double steerAngleDegrees = steerAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360;
        double curSteerAngleRadians = Math.toRadians(steerAngleDegrees);

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(new Rotation2d(curSteerAngleRadians));

        // The output of the steerAnglePID becomes the steer motor rpm reference.
        double steerMotorRpm = steerAnglePID.calculate(steerAngleDegrees,
                desiredState.angle.getDegrees());

        steerMotorVelocityPID.setSetpoint(steerMotorRpm, ControlType.kVelocity);

        // Always-on steer diagnostics -- unlike the drive numbers below, these aren't gated
        // behind logValues, since teleop's drive() never passes true and this is exactly what
        // you need to tell "steer PID isn't tracking" apart from "input isn't reaching the module".
        SmartDashboard.putNumber(name + " SteerAngleActualDeg", steerAngleDegrees);
        SmartDashboard.putNumber(name + " SteerAngleTargetDeg", desiredState.angle.getDegrees());
        SmartDashboard.putNumber(name + " SteerMotorRpmCommand", steerMotorRpm);

        double driveMotorRpm = driveRpmFromSpeed(desiredState.speedMetersPerSecond);

        if (logValues) {
            double driveSpeed = driveNEOMotorEncoder.getVelocity();
            SmartDashboard.putNumber(name + " DriveSpeedMetersPerSecond", desiredState.speedMetersPerSecond);
            SmartDashboard.putNumber(name + " DriveMotorRpmCommand", driveMotorRpm);
            SmartDashboard.putNumber(name + " DriveMotorSpeed", driveSpeed);
        }

        driveMotorVelocityPID.setSetpoint(driveMotorRpm, ControlType.kVelocity);
    }

    /**
     * Returns the required motor rpm from the desired wheel speed in meters/second
     * 
     * @param speedMetersPerSecond
     * @return rpm of the motor
     */
    public double driveRpmFromSpeed(double speedMetersPerSecond) {
        var rpm = speedMetersPerSecond * 60.0 / DriveConstants.WHEEL_CIRCUMFERENCE / DriveConstants.DRIVE_GEAR_RATIO;
        return rpm; // THIS WAS *-1 BUT MADE INVETED INSTED.
    }

    /**
     * Returns the wheel speed in meters/second calculated from the drive motor rpm.
     * 
     * @param rpm
     * @return wheelSpeed
     */
    public double speedFromDriveRpm(double rpm) {
        var speedMetersPerSecond = rpm * DriveConstants.DRIVE_GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE / 60.0;
        return speedMetersPerSecond;  // THIS WAS *-1 BUT MADE INVETED INSTED.
    }

    public SwerveModulePosition getPosition() {
        double distance = driveNEOMotorEncoder.getPosition();
        return new SwerveModulePosition(distance, new Rotation2d(Math.toRadians(steerAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360)));
    }
}