package frc.robot.subsystems.vision

import edu.wpi.first.networktables.NetworkTableInstance
import org.ghrobotics.lib.commands.FalconSubsystem

object VisionSubsystem : FalconSubsystem() {
    val table = NetworkTableInstance.getDefault().getTable("limelight")
    val hasTargets = table.getEntry('tv') // 0 or 1
    val xOffset = table.getEntry('tx') // horizontal offset from -27 to +27 degrees
    val yOffset = table.getEntry('ty') // vertical offset from -20.5 to +20.5 degrees
    val targetArea = table.getEntry('ta') // 0% to 100% of image

//    fun setLEDOff() {
//    }
I

}