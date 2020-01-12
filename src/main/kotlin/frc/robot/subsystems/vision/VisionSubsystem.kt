package frc.robot.subsystems.vision

import edu.wpi.first.networktables.NetworkTableInstance
import org.ghrobotics.lib.commands.FalconSubsystem

object VisionSubsystem : FalconSubsystem() {
    val table = NetworkTableInstance.getDefault().getTable("limelight")

    ///////////////////////
    // Reading Functions //
    ///////////////////////
    fun getHasTargets() {
        return table.getEntry('tv')
    } // 0 or 1

    fun getXOffset() {
        return table.getEntry('tx')
    }  // horizontal offset from -27 to +27 degrees

    fun getYOffset() {
        return table.getEntry('ty')
    } // vertical offset from -20.5 to +20.5 degrees

    fun getTargetArea() {
        table.getEntry('ta')
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

    fun setPipeline(pipeline) {
        if (pipeline >= 0 && pipeline <= 9) { // in range
            table.getEntry("pipeline").setValue(pipeline)
        } else {
            return -1
        }
    }





}