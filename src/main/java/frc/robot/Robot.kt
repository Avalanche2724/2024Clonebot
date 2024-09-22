// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler

class Robot : TimedRobot() {
    private val robotContainer = RobotContainer(this)
    private var autonomousCommand = robotContainer.autonomousCommand

    override fun robotInit() {
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_Language,
            FRCNetComm.tInstances.kLanguage_Kotlin
        )
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledPeriodic() {
    }

    override fun autonomousInit() {
        autonomousCommand = robotContainer.autonomousCommand.also { it.schedule() }
    }

    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {
        autonomousCommand.cancel()
    }

    override fun teleopPeriodic() {
        robotContainer.updateVision()
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun simulationPeriodic() {
        robotContainer.simulationPeriodic()
    }
}
