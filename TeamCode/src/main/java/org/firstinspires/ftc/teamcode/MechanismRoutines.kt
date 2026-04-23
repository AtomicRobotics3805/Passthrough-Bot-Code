package org.firstinspires.ftc.teamcode

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import org.firstinspires.ftc.teamcode.GlobalVars.SHOOT_1_BALL_DELAY

object MechanismRoutines {


    val Shoot3Routine = SequentialGroup(
        Shooter.start,
        Gate.open,
        Intake.start,
        Delay(SHOOT_1_BALL_DELAY * 3),
        Intake.stop,
        Gate.close,
        Shooter.stop
    )

    val Shoot1Routine = SequentialGroup(
        Shooter.start,
        Gate.open,
        Intake.start,
        Delay(SHOOT_1_BALL_DELAY),
        Intake.stop,
        Gate.close,
        Shooter.stop
    )

    val BeginShoot = SequentialGroup(
        Shooter.start,
        Gate.open,
        Intake.start
    )

    val EndShoot = SequentialGroup(
        Intake.stop,
        Gate.close,
        Shooter.stop
    )

}