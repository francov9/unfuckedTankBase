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

  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

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
            () -> baseVelX.getAsDouble() * -0.5, () -> baseVelY.getAsDouble() * 0.5));

    // m_driverController
    //     .leftBumper()
    //     .whileTrue(m_ArmSubsystem.armUpCommand())
    //     .whileFalse(m_ArmSubsystem.armNeutralCommand());
    m_driverController.b().onTrue(m_ArmSubsystem.armUpCommand());
    m_driverController.y().whileTrue(m_ArmSubsystem.armDownCommand());
    m_driverController
        .a()
        .whileTrue(m_GripperSubsystem.gripperCloseCommand())
        .whileFalse(m_GripperSubsystem.gripperOpenCommand());
    m_driverController.axisGreaterThan(3, 0.5).whileTrue(m_RollerSubsystem.rollerForwardCommand());
    m_driverController.axisGreaterThan(2, 0.5).whileTrue(m_RollerSubsystem.rollerBackwardCommand());
    m_driverController.x().whileTrue(m_RollerSubsystem.stopRollerCommand());
  }
}
