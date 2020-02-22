package frc.robot

object Ports {


    const val kPcmId = 8
    const val kPcmId2 = 9


    // MOTORS
    const val intakeMotorId = 13


    const val intakeMotorId = 13
    const val collectorAgitatorId = 12

    val shooterGearboxIds = listOf(9, 10)
    const val shooterHoodId = 12
    const val bumperGrabberId = 11

    // SOLENOIDS

    val chonkIntake = listOf(2,3)
    const val collectorAgitatorId = 12

    val shooterGearboxIds = listOf(9, 10)
    const val shooterHoodId = 14
    const val bumperGrabberId = 12

    // SOLENOIDS

    val intakeSolenoid = listOf(2, 7, 6, 7) // outer and inner

    val shooterShifterSolenoid = listOf(5, 1)
    val armSolenoid = listOf(3, 4) // TODO make real

    val bumperGrabberSolenoid = listOf(0, 1)

    const val hoodEncoderPort = 4
}
