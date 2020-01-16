package frc.robot.subsystems.sensors
//package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;

object Sensors {

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

     class Robot : TimedRobot() {
        /**
         * Change the I2C port below to match the connection of your color sensor
         */
        private val i2cPort = I2C.Port.kOnboard

        /**
         * A Rev Color Sensor V3 object is constructed with an I2C port as a
         * parameter. The device will be automatically initialized with default
         * parameters.
         */
        private val m_colorSensor = ColorSensorV3(i2cPort)

        override fun robotPeriodic() {
            /**
             * The method GetColor() returns a normalized color value from the sensor and can be
             * useful if outputting the color to an RGB LED or similar. To
             * read the raw color, use GetRawColor().
             *
             * The color sensor works best when within a few inches from an object in
             * well lit conditions (the built in LED is a big help here!). The farther
             * an object is the more light from the surroundings will bleed into the
             * measurements and make it difficult to accurately determine its color.
             */
            val detectedColor = m_colorSensor.getColor()

            /**
             * The sensor returns a raw IR value of the infrared light detected.
             */
            val IR = m_colorSensor.getIR()

            /**
             * Open Smart Dashboard or Shuffleboard to see the color detected by the
             * sensor.
             */
            SmartDashboard.putNumber("Red", detectedColor.red)
            SmartDashboard.putNumber("Green", detectedColor.green)
            SmartDashboard.putNumber("Blue", detectedColor.blue)
            SmartDashboard.putNumber("IR", IR)

            val proximity = m_colorSensor.getProximity()

            SmartDashboard.putNumber("Proximity", proximity.toDouble())


        }
    }

    fun update() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

}