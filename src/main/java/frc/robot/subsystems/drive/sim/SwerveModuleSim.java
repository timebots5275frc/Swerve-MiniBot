package frc.robot.subsystems.drive.sim;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.drive.SwerveModule;

/**
 * Physics-backed simulation companion for one real {@link SwerveModule}.
 *
 * <p>Every loop: read what the real Spark closed loops are currently commanding (via
 * {@code getAppliedOutput()}), step a DCMotorSim for the drive motor and one for the steer
 * motor, then write the results back into that module's simulated Spark encoders and CANcoder.
 * The real {@link SwerveModule} code reads those back exactly like it would on hardware --
 * this class never touches SwerveModule's control logic, only its sim-state hooks.
 */
public class SwerveModuleSim {

    // Both motors are NEOs on this robot -- see SwerveModule.
    private static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);
    private static final DCMotor STEER_GEARBOX = DCMotor.getNEO(1);

    // Motor rotations per output (wheel / steer-shaft) rotation -- inverse of DriveConstants'
    // output/motor ratios, since createDCMotorSystem wants a reduction expressed as
    // motor-speed-over-output-speed.
    private static final double DRIVE_GEARING = 1.0 / DriveConstants.DRIVE_GEAR_RATIO;
    private static final double STEER_GEARING = 1.0 / DriveConstants.STEER_GEAR_RATIO;

    // Rough moment-of-inertia guesses (kg*m^2). These only affect how quickly the sim
    // accelerates/settles, not correctness -- tune if the sim feels too snappy or too sluggish.
    private static final double DRIVE_MOI_KG_METERS_SQUARED = 0.025;
    private static final double STEER_MOI_KG_METERS_SQUARED = 0.004;

    private final DCMotorSim driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, DRIVE_MOI_KG_METERS_SQUARED, DRIVE_GEARING),
            DRIVE_GEARBOX);
    private final DCMotorSim steerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(STEER_GEARBOX, STEER_MOI_KG_METERS_SQUARED, STEER_GEARING),
            STEER_GEARBOX);

    private final SparkMaxSim driveMotorSim;
    private final SparkMaxSim steerMotorSim;
    private final CANcoderSimState steerEncoderSim;

    public SwerveModuleSim(SwerveModule module) {
        driveMotorSim = new SparkMaxSim(module.driveMotor, DRIVE_GEARBOX);
        steerMotorSim = new SparkMaxSim(module.steerMotor, STEER_GEARBOX);
        steerEncoderSim = module.steerAngleEncoder.getSimState();
    }

    /**
     * Advances this module's physics by one robot loop.
     *
     * <p>Order matters: read what the real Spark closed loops are currently commanding, step
     * the physics, then write the result back into the simulated sensors -- same shape as REV's
     * own sim examples.
     *
     * @param busVoltageVolts current simulated battery/bus voltage
     * @param dtSeconds       timestep, normally one robot loop (20 ms)
     */
    public void update(double busVoltageVolts, double dtSeconds) {
        // Drive
        driveSim.setInputVoltage(driveMotorSim.getAppliedOutput() * busVoltageVolts);
        driveSim.update(dtSeconds);
        driveMotorSim.iterate(driveSim.getAngularVelocityRPM() * DRIVE_GEARING, busVoltageVolts, dtSeconds);

        // Steer
        steerSim.setInputVoltage(steerMotorSim.getAppliedOutput() * busVoltageVolts);
        steerSim.update(dtSeconds);
        steerMotorSim.iterate(steerSim.getAngularVelocityRPM() * STEER_GEARING, busVoltageVolts, dtSeconds);

        // steerSim's OUTPUT shaft (post-gearing) IS the wheel/CANcoder shaft, so its position
        // and velocity feed the CANcoder sim state directly -- no extra gearing applied here.
        steerEncoderSim.setRawPosition(steerSim.getAngularPositionRotations());
        steerEncoderSim.setVelocity(steerSim.getAngularVelocityRPM() / 60.0);
    }

    /** Total current draw of this module's two motors, for battery sag simulation. */
    public double getCurrentDrawAmps() {
        return Math.abs(driveSim.getCurrentDrawAmps()) + Math.abs(steerSim.getCurrentDrawAmps());
    }
}
