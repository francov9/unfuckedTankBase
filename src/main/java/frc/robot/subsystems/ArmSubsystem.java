package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.utils.SysId;

public class ArmSubsystem extends AdvancedSubsystem {

  private final TalonFX armMotor = new TalonFX(7, Constants.rioCan.rio);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

  private final double zeroPos = armMotor.getPosition().getValueAsDouble();
  private final double upperBound = zeroPos + 3.4;

  private double kAngle = 0;

  private StatusSignal<Angle> armPosition = armMotor.getPosition();

  private StatusSignal<AngularVelocity> armVelocity = armMotor.getVelocity();
  private final VoltageOut _armVoltageSetter = new VoltageOut(0);


  private double currentPositionDeg;

  private final SysIdRoutine rightTrack =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(0.5),
              Volts.of(4),
              Seconds.of(5),
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Voltage volts) ->
                  armMotor.setControl(_armVoltageSetter.withOutput(volts)),
              null,
              this));

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
    motionMagicConfigs.MotionMagicCruiseVelocity = 1;
    motionMagicConfigs.MotionMagicAcceleration = 160;
    motionMagicConfigs.MotionMagicJerk = 1600;

    armMotor.setNeutralMode(NeutralModeValue.Brake);

    armMotor.getConfigurator().apply(talonFXConfigs);
    // armPosition = armMotor.getPosition();

    SysId.displayRoutine("Arm Flywheel", rightTrack);
  }

  public void periodic() {
    armPosition.refresh();
    // currentPositionDeg = armPosition.getValue().in(Units.Degrees);
    super.periodic();
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
  //   // return run(() ->
  // armMotor.setControl(positionRequest.withPosition(kAngle))).withName("Down");
  // }

  // public Command armUpCommand(double zeroPos) {
  //   System.out.println("up");

  //   return run(() -> armMotor.setPosition(zeroPos - 0.1));
  //   //armMotor.setPosition(0);
  //   // kAngle-=0.01;
  //   // return run(() ->
  // armMotor.setControl(positionRequest.withPosition(kAngle))).withName("Up");
  // }

  // public double armZero(){
  //   return armMotor.getPosition().getValueAsDouble();
  // }
}
