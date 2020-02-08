package frc.robot

object Ports {

    const val kPcmId = 9

    // MOTORS

    const val intakeMotorId = 12
    const val collectorAgitatorId = 12

    val shooterGearboxIds = listOf(12, 12)
    const val shooterHoodId = 12
    const val bumperGrabberId = 12

    // SOLENOIDS

    val intakeSolenoid = listOf(2, 7, 7, 6) // outer and inner

    val shooterShifterSolenoid = listOf(4, 5)
    val armSolenoid = listOf(6, 7)

    val bumperGrabberSolenoid = listOf(0, 1)

    const val hoodEncoderPort = 4
}
