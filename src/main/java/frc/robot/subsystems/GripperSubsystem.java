// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;

public class GripperSubsystem extends AdvancedSubsystem {
  public GripperSubsystem() {}

  Servo gripperServo = new Servo(1);

  public Command gripperCloseCommand() {
    return run(() -> gripperServo.set(1)).withName("Closed");
  }

  public Command gripperOpenCommand() {
    return run(() -> gripperServo.set(0)).withName("Open");
  }

  public Command gripperServoStopCommand() {
    return run(() -> gripperServo.set(0.5));
  }
}
