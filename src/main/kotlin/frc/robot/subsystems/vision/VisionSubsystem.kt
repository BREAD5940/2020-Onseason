package frc.robot.subsystems.vision

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import org.ghrobotics.lib.commands.FalconSubsystem

object VisionSubsystem : FalconSubsystem() {
    val table = NetworkTableInstance.getDefault().getTable("limelight")

    ///////////////////////
    // Reading Functions //
    ///////////////////////
    fun getHasTargets(): Boolean {
        return (table.getEntry("tv").getNumber(0)) == 1
    }

    fun getXOffset(): Double {
        return table.getEntry("tx").getDouble(0.0)
    }  // horizontal offset from -27 to +27 degrees

    fun getYOffset(): Double {
        return table.getEntry("ty").getDouble(0.0)
    } // vertical offset from -20.5 to +20.5 degrees

    fun getTargetArea(): Double? {
        return table.getEntry("ta").getDouble(0.0)
    } // 0% to 100% of image


    ///////////////////////
    // Writing Functions //
    ///////////////////////

    // LED functions
    fun setLEDOff() {
        table.getEntry("ledMode").setValue(1)
    }

    fun setLEDBlink() {
        table.getEntry("ledMode").setValue(2)
    }

    fun setLEDOn() {
        table.getEntry("ledMode").setValue(3)
    }

    // mode setting functions
    fun setAsVisionProcessor() {
        table.getEntry("camMode").setValue(0)
    }

    fun setAsDriverCamera() {
        table.getEntry("camMode").setValue(1)
    }

    fun setPipeline(pipeline: Int) {
        if (pipeline >= 0 && pipeline <= 9) { // in range
            table.getEntry("pipeline").setValue(pipeline)
        } else {
            return
        }
    }


}