package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;

public class SysIdRoutine extends edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine {
  private final BooleanSupplier forwardLimitSupplier;
  private final BooleanSupplier backwardLimitSupplier;
  private final Subsystem subsystem;

  public SysIdRoutine(Config config, Mechanism mechanism, Subsystem subsystem) {
    super(config, mechanism);

    this.backwardLimitSupplier = () -> false;
    this.forwardLimitSupplier = () -> false;
    this.subsystem = subsystem;
  }

  public SysIdRoutine(
      Config config,
      Mechanism mechanism,
      Subsystem subsystem,
      BooleanSupplier forwardLimitSupplier,
      BooleanSupplier backwardLimitSupplier) {
    super(config, mechanism);

    this.backwardLimitSupplier = backwardLimitSupplier;
    this.forwardLimitSupplier = forwardLimitSupplier;
    this.subsystem = subsystem;
  }

  private BooleanSupplier limitSupplier(Direction direction) {
    if (direction == Direction.kForward) {
      return forwardLimitSupplier;
    } else {
      return backwardLimitSupplier;
    }
  }

  @Override
  public Command quasistatic(Direction direction) {
    Command command =
        super.quasistatic(direction)
            .until(limitSupplier(direction))
            .withName("SysID Quasistatic: " + direction.toString());
    command.addRequirements(subsystem);
    return command;
  }

  @Override
  public Command dynamic(Direction direction) {
    Command command =
        super.dynamic(direction)
            .until(limitSupplier(direction))
            .withName("SysID Dynamic: " + direction.toString());
    command.addRequirements(subsystem);
    return command;
  }
}
