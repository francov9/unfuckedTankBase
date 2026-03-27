package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.robot.Constants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class ArmSubsystem extends AdvancedSubsystem {

  
  private final TalonFX armMotor = new TalonFX(7, Constants.rioCan.rio);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  

  private final double zeroPos = armMotor.getPosition().getValueAsDouble();
  private final double upperBound = zeroPos + 3.8;

  private double kAngle = 0;
  private StatusSignal<Angle> armPosition = armMotor.getPosition();
  private double currentPositionDeg;

  public ArmSubsystem() {
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.14;
    slot0Configs.kV = 0.939;
    slot0Configs.kA = 0.1;
    slot0Configs.kG = 0.435;

    slot0Configs.kP = 28;
    slot0Configs.kD = 0.2;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = .5;
    motionMagicConfigs.MotionMagicAcceleration = 160;
    motionMagicConfigs.MotionMagicJerk = 1600;

    armMotor.setNeutralMode(NeutralModeValue.Brake);

    armMotor.getConfigurator().apply(talonFXConfigs);
    armPosition = armMotor.getPosition();
  }

  public void periodic() {
    armPosition.refresh();
    currentPositionDeg = armPosition.getValue().in(Units.Degrees);
  }


  public Command armUpCommand() {
    return run(() -> armMotor.setControl(positionRequest.withPosition(upperBound)));
  }

  public Command armDownCommand() {
    return run(() -> armMotor.setControl(positionRequest.withPosition(zeroPos)));
  }




  // public Command armDownCommand(double zeroPos) {
  //   System.out.println("down");



  //   return run(() -> armMotor.setPosition(zeroPos + 0.1));

  //   //armMotor.setPosition(0);
  //   // kAngle+=0.01;
  //   // return run(() -> armMotor.setControl(positionRequest.withPosition(kAngle))).withName("Down");
  // }

  // public Command armUpCommand(double zeroPos) {
  //   System.out.println("up");


  //   return run(() -> armMotor.setPosition(zeroPos - 0.1));
  //   //armMotor.setPosition(0);
  //   // kAngle-=0.01;
  //   // return run(() -> armMotor.setControl(positionRequest.withPosition(kAngle))).withName("Up");
  // }

  // public double armZero(){
  //   return armMotor.getPosition().getValueAsDouble();
  // }
}
