package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.InputStream;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TankBase;

public class RobotContainer {
  private final TankBase m_TankBase = new TankBase();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final GripperSubsystem m_GripperSubsystem = new GripperSubsystem();
  private final RollerSubsystem m_RollerSubsystem = new RollerSubsystem();
  // private final CommandXboxController m_driverController =
  // new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    // Set the default command for the drivetrain.
    // This runs constantly unless another command requires the drivetrain.

    configureBindings();
  }

  private void configureBindings() {
    InputStream baseVelX = InputStream.of(m_driverController::getRightX).deadband(0.02, 1);
    InputStream baseVelY = InputStream.of(m_driverController::getLeftY).deadband(0.02, 1).negate();
    // Reset Pose when Y is pressed
    m_TankBase.setDefaultCommand(
        m_TankBase.driveCommand(
            () -> baseVelY.getAsDouble() * 0.5, () -> baseVelX.getAsDouble() * 0.5));

    m_driverController
        .rightBumper()
        .onTrue(m_ArmSubsystem.armDownCommand())
        .onFalse(m_ArmSubsystem.armUpCommand());
    m_driverController
        .leftBumper()
        .onTrue(m_GripperSubsystem.gripperCloseCommand())
        .onFalse(m_GripperSubsystem.gripperOpenCommand());
    m_driverController
        .axisGreaterThan(3, 0.5)
        .onTrue(m_RollerSubsystem.rollerForwardCommand())
        .onFalse(m_RollerSubsystem.rollerBackwardCommand());
  }
}
