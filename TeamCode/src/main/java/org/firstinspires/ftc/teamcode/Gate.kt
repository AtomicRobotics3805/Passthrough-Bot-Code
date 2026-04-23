package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.util.RobotLog
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

@Configurable
object Gate : Subsystem {


    private val servo = ServoEx("gate")

    @JvmField
    var closePos = 0.0
    @JvmField
    var openPos = 0.0

    val open = SetPosition(servo, openPos).requires(this)

    val close = SetPosition(servo, closePos).requires(this)

    override fun periodic() {
        RobotLog.d("Gate Pos: ", if (servo.position == openPos) { "open" } else { "closed" })
    }


}