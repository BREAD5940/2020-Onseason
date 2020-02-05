package frc.robot

object Ports {

    const val kPcmId = 8

    // MOTORS

    val driveAngleIds = listOf(1, 3, 5, 7)
    val driveDriveIds = listOf(2, 4, 6, 8)

    const val intakeMotorId = 13
    const val collectorAgitatorId = 12

    val shooterGearboxIds = listOf(15, 14)
    const val shooterHoodId = 16
    const val bumperGrabberId = 11

    // SOLENOIDS

    val intakeSolenoid = listOf(0, 1)

    val shooterShifterSolenoid = listOf(4, 5)
    val armSolenoid = listOf(6, 7)

    val bumperGrabberSolenoid = listOf(0, 1)

    const val hoodEncoderPort = 4
}
