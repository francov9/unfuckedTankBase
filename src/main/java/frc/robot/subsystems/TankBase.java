package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.AdvancedSubsystem;
import frc.robot.Robot;

public class TankBase extends AdvancedSubsystem {
  private final CANBus rio = new CANBus("rio");
  private final TalonFX lMotor = new TalonFX(6, rio);
  private final TalonFX rMotor = new TalonFX(2, rio);

  // Use WPILib's DifferentialDrive for standard Arcade Drive math
  private final DifferentialDrive m_drive = new DifferentialDrive(lMotor::set, rMotor::set);

  private final AnalogGyro m_gyro = new AnalogGyro(1); // Will change for a Pigeon at some point
  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  // Physical Constants
  private static final double kGearRatio = 8.31;
  private static final double kWheelRadiusMeters = Units.inchesToMeters(2.8);

  static final double KvLinear = 1.98;
  static final double KaLinear = 0.2;
  static final double KvAngular = 1.5;
  static final double KaAngular = 0.3;

  private final DifferentialDriveOdometry m_odometry;

  private Notifier _simNotifier;

  // @Logged(name = "Pose")
  public Pose2d pose = new Pose2d();




  // private final SysIdSwerveTranslation _translationCharacterization = new SysIdSwerveTranslation();
  // private final SysIdSwerveSteerGains _steerCharacterization = new SysIdSwerveSteerGains();
  // private final SysIdSwerveRotation _rotationCharacterization = new SysIdSwerveRotation();

  // private final SysIdRoutine _translationRoutine =
  //     new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //             null, // Use default ramp rate (1 V/s)
  //             Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
  //             null, // Use default timeout (10 s)
  //             // Log state with SignalLogger class
  //             state -> SignalLogger.writeString("state", state.toString())),
  //         new SysIdRoutine.Mechanism(
  //             output -> setControl(_translationCharacterization.withVolts(output)), null, this));

  // private final SysIdRoutine _steerRoutine =
  //     new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //             null, // Use default ramp rate (1 V/s)
  //             Volts.of(7), // Use dynamic voltage of 7 V
  //             null, // Use default timeout (10 s)
  //             // Log state with SignalLogger class
  //             state -> SignalLogger.writeString("state", state.toString())),
  //         new SysIdRoutine.Mechanism(
  //             volts -> setControl(_steerCharacterization.withVolts(volts)), null, this));

  // private final SysIdRoutine _rotationRoutine =
  //     new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //             /* This is in radians per second², but SysId only supports "volts per second" */
  //             Volts.of(Math.PI / 6).per(Second),
  //             /* This is in radians per second, but SysId only supports "volts" */
  //             Volts.of(Math.PI),
  //             null, // Use default timeout (10 s)
  //             // Log state with SignalLogger class
  //             state -> SignalLogger.writeString("state", state.toString())),
  //         new SysIdRoutine.Mechanism(
  //             output -> {
  //               // output is actually radians per second, but SysId only supports "volts"
  //               setControl(_rotationCharacterization.withRotationalRate(output.in(Volts)));
  //               // also log the requested output for SysId
  //               SignalLogger.writeDouble("rotational_rate", output.in(Volts));
  //             },
  //             null,
  //             this));




  // We ONLY need DifferentialDrivetrainSim. DCMotorSim only need for one motor sim
  private final DifferentialDrivetrainSim drivetrainSim =
      new DifferentialDrivetrainSim(
          LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
          DCMotor.getKrakenX60(1),
          kGearRatio,
          Units.inchesToMeters(15),
          kWheelRadiusMeters,
          null);

  public TankBase() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Invert the right side so positive power moves both sides forward
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 4;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.5;
    slot0Configs.kS = 0.3;
    config.withSlot0(slot0Configs);

    lMotor.getConfigurator().apply(config);
    rMotor.getConfigurator().apply(config);

    // Initialize Odometry using our Rotations -> Meters conversion
    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(),
            rotationsToMeters(lMotor.getPosition().getValueAsDouble()),
            rotationsToMeters(rMotor.getPosition().getValueAsDouble()));
    m_gyroSim.setAngle(0);

    if (Robot.isSimulation()) {
      rMotor.setPosition(0);
      lMotor.setPosition(0);
      startSimThread();
    }
  }

  // Helper methods for math conversions
  private double rotationsToMeters(double rotations) {
    return rotations * (2 * Math.PI * kWheelRadiusMeters) / kGearRatio;
  }

  private double metersToRotations(double meters) {
    return (meters * kGearRatio) / (2 * Math.PI * kWheelRadiusMeters);
  }

  // Helper for velocity
  private double metersPerSecToRPS(double metersPerSec) {
    return (metersPerSec * kGearRatio) / (2 * Math.PI * kWheelRadiusMeters);
  }

  public Command driveCommand(DoubleSupplier speed, DoubleSupplier turn) {
    return run(() -> m_drive.arcadeDrive(speed.getAsDouble(), turn.getAsDouble()))
        .withName("Drive");
  }

  public Command brake() {
    return run(() -> lMotor.set(0)).alongWith(run(() -> rMotor.set(0))).withName("Brake");
  }

  @Override
  public void periodic() {
    // Update odometry converting rotations to meters
    m_odometry.update(
        m_gyro.getRotation2d(),
        rotationsToMeters(lMotor.getPosition().getValueAsDouble()),
        rotationsToMeters(rMotor.getPosition().getValueAsDouble()));
    pose = m_odometry.getPoseMeters();
    DogLog.log("TankBase/Position", pose);
    // DogLog.log("TankBase/CurrentComma", driving);
    super.periodic();
  }

  private void startSimThread() {
    _simNotifier =
        new Notifier(
            () -> {
              final double batteryVolts = RobotController.getBatteryVoltage();

              var LtalonFXSim = lMotor.getSimState();
              var RtalonFXSim = rMotor.getSimState();

              LtalonFXSim.setSupplyVoltage(batteryVolts);
              RtalonFXSim.setSupplyVoltage(batteryVolts);

              // 1. Get applied voltage from the simulated Talons
              double leftVolts = LtalonFXSim.getMotorVoltageMeasure().in(Volts);
              double rightVolts = RtalonFXSim.getMotorVoltageMeasure().in(Volts);

              // 2. Feed voltages into Drivetrain physics simulator
              drivetrainSim.setInputs(leftVolts, rightVolts);
              drivetrainSim.update(0.02);

              // 3. Get simulated physical state (meters) and convert back to Talon state
              // (rotations)
              LtalonFXSim.setRawRotorPosition(
                  metersToRotations(drivetrainSim.getLeftPositionMeters()));
              LtalonFXSim.setRotorVelocity(
                  metersPerSecToRPS(drivetrainSim.getLeftVelocityMetersPerSecond()));

              RtalonFXSim.setRawRotorPosition(
                  metersToRotations(drivetrainSim.getRightPositionMeters()));
              RtalonFXSim.setRotorVelocity(
                  metersPerSecToRPS(drivetrainSim.getRightVelocityMetersPerSecond()));

              // Update gyro
              m_gyroSim.setAngle(-drivetrainSim.getHeading().getDegrees());
            });

    _simNotifier.setName("Sim Thread");
    _simNotifier.startPeriodic(1 / Hertz.of(200).in(Hertz));
  }

  @Override
  public void close() {
    lMotor.close();
    rMotor.close();
  }
}
