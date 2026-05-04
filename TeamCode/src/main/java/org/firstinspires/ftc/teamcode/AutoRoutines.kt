package org.firstinspires.ftc.teamcode

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.FollowPath

object AutoRoutines {

    val nearNineAutoRoutine = SequentialGroup(
        Shooter.stop,
        FollowPath(Paths.goalStartToCloseShoot, true),
        MechanismRoutines.Shoot3Routine,
        FollowPath(Paths.closeShootToSpike1, true),
        Intake.start,
        FollowPath(Paths.intakeSpike1, true),
        Intake.stop,
        FollowPath(Paths.spike1ToDump, true),
        Delay(1.0),
        FollowPath(Paths.dumpToCloseShoot, true),
        MechanismRoutines.Shoot3Routine,
        FollowPath(Paths.closeShootToSpike2, true),
        Intake.start,
        FollowPath(Paths.intakeSpike2, true),
        Intake.stop,
        FollowPath(Paths.spike2ToDump, true),
        Delay(1.0),
        FollowPath(Paths.dumpToCloseShoot, true),
        MechanismRoutines.Shoot3Routine,
        FollowPath(Paths.closeShootToClosePark, true)
    )

}