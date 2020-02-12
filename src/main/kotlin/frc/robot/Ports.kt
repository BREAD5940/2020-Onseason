package frc.robot

object Ports {

    const val kPcmId = 9
    const val kPcmId2 = 8

    // MOTORS

    const val intakeMotorId = 13

    const val collectorAgitatorId = 12

    val shooterGearboxIds = listOf(9, 10)
    const val shooterHoodId = 12
    const val bumperGrabberId = 12

    // SOLENOIDS

    val intakeSolenoid = listOf(2, 7, 7, 6) // outer and inner

    val shooterShifterSolenoid = listOf(4, 5)
    val armSolenoid = listOf(3, 1) // TODO make real

    val bumperGrabberSolenoid = listOf(0, 1)

    const val hoodEncoderPort = 4
}
