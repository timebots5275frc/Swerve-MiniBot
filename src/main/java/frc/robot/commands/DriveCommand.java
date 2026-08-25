// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.customTypes.Math.Vector2;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.subsystems.Input;

/**
 * Teleop swerve drive.
 *
 * Translation pipeline order (this order matters):
 *   raw stick x/y -> split into magnitude + direction -> response curve on magnitude ->
 *   slew limit on magnitude -> recombine with direction -> scale to velocity -> drive
 *
 * Translation is deliberately processed as magnitude+direction rather than x/y independently:
 * curving or slew-limiting x and y separately and recombining them afterward is
 * direction-dependent (a diagonal push ends up faster than a same-distance single-axis push).
 * Rotation has no such concern since it's already a single scalar axis.
 *
 * Note that the slew limiters run on NORMALIZED (-1..1) input, not on m/s. That makes
 * JOY_TRANSLATION_RATE_LIMIT / JOY_TURN_RATE_LIMIT mean "fraction of full stick travel per
 * second", which is far easier to tune. A value of 3.0 means full travel in 1/3 second.
 */
public class DriveCommand extends Command {

    /**
     * Response curve blend. 0.0 = fully linear, 1.0 = fully cubic.
     * Pure cubic (1.0) feels dead near center and abrupt near the ends.
     * 0.6-0.8 gives fine low-speed control while still reaching full output smoothly.
     */
    private static final double DRIVE_CURVE = 0.3;
    private static final double TURN_CURVE = 0.3;

    private final SwerveDriveSubsystem drivetrain;
    private final Input input;

    private final SlewRateLimiter srlTranslation =
            new SlewRateLimiter(Constants.JoystickConstants.JOY_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter srlTurn =
            new SlewRateLimiter(Constants.JoystickConstants.JOY_TURN_RATE_LIMIT);

    private boolean fieldRelative;

    /**
     * @param drive         the drive subsystem this command will run on
     * @param input_        the control input for driving
     * @param _fieldRelative whether to drive field-relative
     */
    public DriveCommand(SwerveDriveSubsystem drive, Input input_, boolean _fieldRelative) {
        this.drivetrain = drive;
        this.input = input_;
        this.fieldRelative = _fieldRelative;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Clear stale limiter state left over from the last time this command ran.
        // Without this, the first frame after enable can command a large jump.
        srlTranslation.reset(0);
        srlTurn.reset(0);
    }

    @Override
    public void execute() {
        // Read inputs into locals. Do not mutate the Vector2 that Input handed us --
        // it may be cached or shared with other consumers.
        Vector2 rawMove = input.DriveInput();
        double rawTurn = input.DriveTwist();
        double translationSpeedPercent = input.TranslationSpeedPercent();
        double twistSpeedPercent = input.TwistSpeedPercent();

        // Decompose into magnitude + direction and process the MAGNITUDE only.
        // Curving/slewing x and y independently and recombining them is direction-dependent
        // (a diagonal push ends up faster than a same-distance single-axis push, since
        // combining two independently-shrunk components doesn't shrink the Euclidean length
        // by the same factor as shrinking the length directly). Working in magnitude+direction
        // guarantees speed depends only on how far the stick is pushed from center, never on
        // which way it's pointing.
        double rawMagnitude = Math.min(rawMove.magnitude(), 1.0); // clamp square-stick corners (e.g. x=1,y=1) to the unit circle
        Vector2 direction = rawMagnitude > 1e-6 ? rawMove.normalized() : Vector2.zero;

        double turn = curve(rawTurn, TURN_CURVE);
        double magnitude = curve(rawMagnitude, DRIVE_CURVE);

        // Slew limit in normalized units. Called EVERY loop, including when the
        // stick is centered, so the limiter ramps down to zero instead of snapping
        // and so its internal state never goes stale.
        magnitude = srlTranslation.calculate(magnitude);
        turn = srlTurn.calculate(turn);

        Vector2 moveNormalized = direction.times(magnitude);

        // Scale to real velocities.
        Vector2 translationVelocity = moveNormalized.times(
                translationSpeedPercent
                * Constants.DriveConstants.MAX_DRIVE_SPEED
                * Constants.JoystickConstants.JOY_INPUT_VELOCITY_MULT);

        double rotationVelocity = turn
                * twistSpeedPercent
                * Constants.DriveConstants.MAX_TWIST_RATE
                * Constants.JoystickConstants.JOY_INPUT_ROTATION_VELOCITY_MULT;

        rotationVelocity = MathUtil.clamp(
                rotationVelocity,
                -Constants.DriveConstants.MAX_TWIST_RATE,
                Constants.DriveConstants.MAX_TWIST_RATE);

        drivetrain.drive(translationVelocity.x, translationVelocity.y, rotationVelocity, fieldRelative);
    }

    /**
     * Blend between linear and cubic response. Sign is always preserved.
     *
     * @param value    normalized input, -1..1
     * @param cubicness 0 = linear, 1 = cubic
     */
    private static double curve(double value, double cubicness) {
        return cubicness * (value * value * value) + (1.0 - cubicness) * value;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, fieldRelative);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void setFieldRelative(boolean bool) {
        this.fieldRelative = bool;
    }
}