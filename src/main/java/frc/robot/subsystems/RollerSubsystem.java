// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase {

  TalonFX rollerMotor = new TalonFX(11, Constants.rioCan.rio);

  /** Creates a new RollerSubsystem. */
  public RollerSubsystem() {}

  public void rollerBackward() {
    rollerMotor.set(0.2);
  }

  public void rollerForward() {
    rollerMotor.set(-0.2);
  }

  public void stopRoller() {
    rollerMotor.set(0);
  }

  public Command rollerBackwardCommand() {
    return run(() -> rollerBackward());
  }

  public Command rollerForwardCommand() {
    return run(() -> rollerForward());
  }

  public Command stopRollerCommand() {
    return run(() -> stopRoller());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
