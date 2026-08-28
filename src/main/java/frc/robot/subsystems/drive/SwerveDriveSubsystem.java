// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.customTypes.Math.Vector2;

public class SwerveDriveSubsystem extends SubsystemBase {
  /** Creates a new SwerveDriveSubsystem. */
  private final SwerveModule leftFrontSwerveModule;
  private final SwerveModule rightFrontSwerveModule;
  private final SwerveModule rightRearSwerveModule;
  private final SwerveModule leftRearSwerveModule;

  public final SwerveModulePosition[] startingSwerveModulePositions;
  private SwerveModulePosition[] currentSwerveModulePositions;


  private final SwerveDriveKinematics kinematics;

  private final SwerveDriveOdometry odometry;
  
  Pigeon2 pigeon2Gyro = new Pigeon2(DriveConstants.PIGEON_2_ID);

private final Field2d field = new Field2d();

  public SwerveDriveSubsystem() {
    // Left/right sides are physical mirror images of each other, so the drive motor invert
    // flag has to flip between them -- see SwerveModule's constructor.
    leftFrontSwerveModule = new SwerveModule(DriveConstants.SwerveCanIDs.LEFT_FRONT_DRIVE_MOTOR_ID, DriveConstants.SwerveCanIDs.LEFT_FRONT_STEER_MOTOR_ID, DriveConstants.SwerveCanIDs.LEFT_FRONT_STEER_ENCODER_ID, false);
    rightFrontSwerveModule = new SwerveModule(DriveConstants.SwerveCanIDs.RIGHT_FRONT_DRIVE_MOTOR_ID, DriveConstants.SwerveCanIDs.RIGHT_FRONT_STEER_MOTOR_ID, DriveConstants.SwerveCanIDs.RIGHT_FRONT_STEER_ENCODER_ID, true);
    rightRearSwerveModule = new SwerveModule(DriveConstants.SwerveCanIDs.RIGHT_REAR_DRIVE_MOTOR_ID, DriveConstants.SwerveCanIDs.RIGHT_REAR_STEER_MOTOR_ID, DriveConstants.SwerveCanIDs.RIGHT_REAR_STEER_ENCODER_ID, true);
    leftRearSwerveModule = new SwerveModule(DriveConstants.SwerveCanIDs.LEFT_REAR_DRIVE_MOTOR_ID, DriveConstants.SwerveCanIDs.LEFT_REAR_STEER_MOTOR_ID, DriveConstants.SwerveCanIDs.LEFT_REAR_STEER_ENCODER_ID, false);

    startingSwerveModulePositions = new SwerveModulePosition[] {leftFrontSwerveModule.getPosition(), rightFrontSwerveModule.getPosition(), rightRearSwerveModule.getPosition(), leftRearSwerveModule.getPosition()};
    currentSwerveModulePositions = startingSwerveModulePositions;

    kinematics = new SwerveDriveKinematics(DriveConstants.LEFT_FRONT_MODULE_POSITION,
                                           DriveConstants.RIGHT_FRONT_MODULE_POSITION,
                                           DriveConstants.RIGHT_REAR_MODULE_POSITION,
                                           DriveConstants.LEFT_REAR_MODULE_POSITION);

    odometry = new SwerveDriveOdometry(kinematics, this.getHeading(), startingSwerveModulePositions);

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentSwerveModulePositions = new SwerveModulePosition[] { leftFrontSwerveModule.getPosition(), rightFrontSwerveModule.getPosition(), rightRearSwerveModule.getPosition(), leftRearSwerveModule.getPosition(), };
    updateOdometry();
    field.setRobotPose(this.getPose());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

      SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, this.getHeading()) : new ChassisSpeeds(xSpeed, ySpeed, rot));

      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_DRIVE_SPEED);

      leftFrontSwerveModule.setDesiredState(swerveModuleStates[0], false, "LF");
      rightFrontSwerveModule.setDesiredState(swerveModuleStates[1], false, "RF");
      rightRearSwerveModule.setDesiredState(swerveModuleStates[2], false, "RR");
      leftRearSwerveModule.setDesiredState(swerveModuleStates[3], false, "LR");

      currentSwerveModulePositions = new SwerveModulePosition[] { leftFrontSwerveModule.getPosition(), rightFrontSwerveModule.getPosition(), rightRearSwerveModule.getPosition(), leftRearSwerveModule.getPosition(), };

      SmartDashboard.putString("Odometry Pos", this.getOdometryPosition().toString());
      SmartDashboard.putString("Odometry Rot", odometry.getPoseMeters().getRotation().toString());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
      return odometry.getPoseMeters();
  }

  public Vector2 getOdometryPosition()
  {
      Pose2d pose = odometry.getPoseMeters();
      return new Vector2(pose.getX(), pose.getY());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
      odometry.update(this.getHeading(), currentSwerveModulePositions);
  }

  /**
   * Resets the odometry Position and Angle to 0.
   */
  public void resetOdometry() {
      System.out.println("resetOdometry");
      odometry.resetPosition(this.getHeading(), currentSwerveModulePositions, new Pose2d());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometryWithPose2d(Pose2d pose) {
      System.out.println("resetOdometryWithPose2d");
      odometry.resetPosition(pose.getRotation(), currentSwerveModulePositions, pose); // imuADIS16470.getRotation2d()
  }

  public void resetPigeon() {

      try{
         pigeon2Gyro.setYaw(0);
      } 
      catch(ArithmeticException e){System.out.println(e);}

  }

  public void setGyroYaw(double degrees) {
      pigeon2Gyro.setYaw(degrees);
  }

  /**
   * Returns the heading of the robot.
   * 
   * @return Rotation2d of the robot heading.
   */
  public Rotation2d getHeading() {

      Rotation2d heading = Rotation2d.fromDegrees(pigeon2Gyro.getYaw().getValueAsDouble());


      return heading;

  }

  public double getGyroYawInDegrees() { return pigeon2Gyro.getYaw().getValueAsDouble(); }

  /**
   * Sets the swerve ModuleStates.
   * 
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_DRIVE_SPEED);

      leftFrontSwerveModule.setDesiredState(desiredStates[0], true, "LF");
      rightFrontSwerveModule.setDesiredState(desiredStates[1], true, "RF");
      rightRearSwerveModule.setDesiredState(desiredStates[2], true, "RR");
      leftRearSwerveModule.setDesiredState(desiredStates[3], true, "LR");
  }

  public void alignWheels() {
      SwerveModuleState desiredStates = new SwerveModuleState(0, new Rotation2d(0));

      leftFrontSwerveModule.setDesiredState(desiredStates, true, "LF");
      rightFrontSwerveModule.setDesiredState(desiredStates, true, "RF");
      rightRearSwerveModule.setDesiredState(desiredStates, true, "RR");
      leftRearSwerveModule.setDesiredState(desiredStates, true, "LR");
  }

  public static Trajectory generateTrajectory(TrajectoryConfig config, List<Pose2d> list) {
      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(list, config);
      return exampleTrajectory;
  }

  /** Modules in the same order used everywhere else in this class: LF, RF, RR, LR. */
  public SwerveModule[] getModules() {
      return new SwerveModule[] { leftFrontSwerveModule, rightFrontSwerveModule, rightRearSwerveModule, leftRearSwerveModule };
  }

  public Pigeon2 getGyro() {
      return pigeon2Gyro;
  }
}
