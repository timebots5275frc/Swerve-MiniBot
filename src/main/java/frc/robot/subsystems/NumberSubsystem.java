// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NumberSubsystem extends SubsystemBase {
  private double number;

  /** Creates a new numberSubsystem. */
  public NumberSubsystem() {
    this.number = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Number", number);
  }

  public double getNumber() {
    return number;
  }

  public void setNumber(double newNumber) {
    this.number = newNumber;
  }
}
