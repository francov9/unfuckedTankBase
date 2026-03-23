package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;

public class ArmSubsystem extends AdvancedSubsystem {

  private final TalonFX armMotor = new TalonFX(6);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

  private double kDownAngle = 0;
  private double kUpAngle = 0;

  public ArmSubsystem() {
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.14;
    slot0Configs.kV = 0.939;
    slot0Configs.kA = 0.002;
    slot0Configs.kG = 0.435;
    slot0Configs.kP = 28;
    slot0Configs.kD = 0.2;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80;
    motionMagicConfigs.MotionMagicAcceleration = 160;
    motionMagicConfigs.MotionMagicJerk = 1600;

    armMotor.getConfigurator().apply(talonFXConfigs);
  }
  @Override
  public void periodic() {
    DogLog.log("Robot/ArmSubsystem/armMotorPosition", armMotor.getPosition().getValueAsDouble());
  }

  public Command armDownCommand() {
    return run(
        () -> armMotor.setControl(positionRequest.withPosition(kDownAngle))).withName("Down");
  }

  public Command armUpCommand() {
    return run(
        () -> armMotor.setControl(positionRequest.withPosition(kUpAngle))).withName("Up");
  }
}