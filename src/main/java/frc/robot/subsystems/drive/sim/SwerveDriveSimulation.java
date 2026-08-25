package frc.robot.subsystems.drive.sim;

import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;

/**
 * Owns one {@link SwerveModuleSim} per real module, plus gyro/battery/Field2d glue.
 *
 * <p>Constructed and ticked only from {@link frc.robot.Robot#simulationInit()} /
 * {@link frc.robot.Robot#simulationPeriodic()} -- nothing in {@code SwerveDriveSubsystem} or
 * {@code SwerveModule} knows this class exists, so real-robot behavior is unaffected.
 */
public class SwerveDriveSimulation {

    private static final double PERIOD_SECONDS = 0.020;

    private final SwerveDriveSubsystem drive;
    private final SwerveModule[] modules;
    private final SwerveModuleSim[] moduleSims;

    // Independent of SwerveDriveSubsystem's own kinematics object -- built from the same
    // DriveConstants module positions, used only to turn simulated module states into a
    // chassis angular velocity for the simulated gyro.
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            DriveConstants.LEFT_FRONT_MODULE_POSITION,
            DriveConstants.RIGHT_FRONT_MODULE_POSITION,
            DriveConstants.RIGHT_REAR_MODULE_POSITION,
            DriveConstants.LEFT_REAR_MODULE_POSITION);

    private final Pigeon2SimState gyroSim;
    private final Field2d field = new Field2d();

    public SwerveDriveSimulation(SwerveDriveSubsystem drive) {
        this.drive = drive;
        this.modules = drive.getModules();

        moduleSims = new SwerveModuleSim[modules.length];
        for (int i = 0; i < modules.length; i++) {
            moduleSims[i] = new SwerveModuleSim(modules[i]);
        }

        gyroSim = drive.getGyro().getSimState();
        SmartDashboard.putData("Field", field);
    }

    /** Call once per robot loop from Robot.simulationPeriodic(). */
    public void update() {
        double busVoltage = RoboRioSim.getVInVoltage();

        SwerveModuleState[] actualStates = new SwerveModuleState[modules.length];
        double totalCurrentAmps = 0;
        for (int i = 0; i < modules.length; i++) {
            moduleSims[i].update(busVoltage, PERIOD_SECONDS);
            // Read back through the REAL SwerveModule code, not the physics sim directly --
            // this exercises the same encoder-reading path the real robot uses.
            actualStates[i] = modules[i].getState();
            totalCurrentAmps += moduleSims[i].getCurrentDrawAmps();
        }

        // No physical gyro to simulate directly -- derive the chassis rotation rate from what
        // the (now-updated) simulated modules are actually reporting, same math a real Pigeon2
        // would be measuring if it could see the wheels turn.
        double omegaDegreesPerSecond =
                Math.toDegrees(kinematics.toChassisSpeeds(actualStates).omegaRadiansPerSecond);
        gyroSim.addYaw(omegaDegreesPerSecond * PERIOD_SECONDS);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps));

        // Deliberately the REAL odometry's pose, not an independently-tracked "true" pose --
        // if a sign or conversion bug exists somewhere, this widget should show it, not hide it.
        field.setRobotPose(drive.getPose());
    }
}
