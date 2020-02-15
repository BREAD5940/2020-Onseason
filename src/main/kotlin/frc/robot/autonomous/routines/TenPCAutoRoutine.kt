package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.auto.paths.TrajectoryFactory
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second

class TenPCAutoRoutine: AutoRoutine() {

    override val duration: SIUnit<Second>
        get() = TrajectoryFactory.tenPointAutoToShieldGenerator.duration +
                TrajectoryFactory.tenPointAutoShieldGeneratorToShoot.duration +
                TrajectoryFactory.tenPointAutoPCFromTrench.duration +
                TrajectoryFactory.tenPointAutoTrenchToShoot.duration

    override val routine: Command
        get() = TODO("not implemented") //To change initializer of created properties use File | Settings | File Templates.


}