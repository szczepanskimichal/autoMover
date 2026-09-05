# Hybrid Snowblower Plan

Status note:

- this path is currently paused
- keep this document as design reference and future planning material
- active implementation focus has moved back to the mower branch until the hardware budget and final snowblower direction are better defined

This document turns the current idea into a practical first hardware plan.

The target is not a full battery-electric snowblower.
The target is a hybrid machine with:

- electric traction on the platform
- a combustion engine used as the energy source
- a mechanical auger driven directly from the combustion side
- ROS 2 used for control, safety state, telemetry, and future automation

That keeps the most power-hungry tool outside the electrical bus while still
preserving the precise control advantages of an electric drive platform.

## Why This Architecture Fits The Project

This repository already models the machine as:

- a drive platform
- an operator input layer
- a work-mode layer
- a tool layer

That maps well to a real hybrid snowblower.

The software can keep electric-drive semantics while the hardware uses a more
budget-friendly mechanical auger path.

## Recommended First Hardware Direction

Use the old mower engine as the power source, but do not use the old mower as
the winter chassis.

Recommended split:

- your existing platform becomes the traction base
- the combustion engine becomes the work-power module
- the auger is driven mechanically from the engine
- the electronics, control computer, and traction motors stay separate

This avoids the weakest part of the donor mower: poor winter traction from a
single driven axle.

## High-Level System Layout

```text
operator keyboard / future remote
-> ROS 2 control stack
-> traction commands, work mode, safety state

combustion engine
-> mechanical belt / chain / clutch path
-> auger

combustion engine
-> generator or alternator stage
-> DC bus
-> battery buffer
-> traction motor controllers
-> left and right traction motors

DC bus
-> DC/DC converter
-> computer, relays, sensors, actuators
```

## What The Machine Must Have

### Traction side

- two-sided driven traction, not single-axle donor behavior
- enough chassis mass for grip
- low center of gravity
- winter tires or chains
- strong motor controllers with predictable low-speed torque

### Mechanical auger side

- direct mechanical power from the combustion engine
- guarded belt or chain drive
- engagement clutch or tensioning mechanism
- overload protection with shear pins or a slip clutch
- service access for clearing jams safely

### Electrical side

- main DC bus, preferably 48 V
- buffer battery pack
- main fuse
- main contactor
- emergency stop circuit
- DC/DC converter for logic and low-voltage loads
- current and voltage measurement

### Safety side

- hardware emergency stop that cuts traction command authority
- hardware path to disengage auger drive
- clear startup state with auger disabled by default
- jam recovery procedure that does not rely only on software

## Why Keep The Auger Mechanical

For this budget and project stage, the auger should remain mechanical because:

- it is the highest-power subsystem
- snow load changes abruptly
- a jam can create large current spikes in an all-electric tool path
- a mechanical path is cheaper and more tolerant of overload events

This does not remove the need for protection.

The correct protection is still mechanical:

- shear bolt
- slip clutch
- belt slip as a weak fallback, not as the main safety strategy

Engine bogging can act as a visible overload signal, but it is not enough by
itself to protect the auger gearbox, shaft, or rotor.

## Suggested Energy Architecture

For the first serious version, treat the electrical side as traction and control
power, not as the primary auger power path.

Recommended direction:

- main bus: 48 V DC
- buffer battery: LiFePO4, roughly 2-4 kWh class for a first real prototype
- traction motors: one per side
- low-voltage rail: 12 V from DC/DC for compute and auxiliaries

The combustion engine can later feed the DC system through a generator stage,
but the repo does not need that hardware to start modeling the machine
correctly.

## Practical Build Stages

### Stage 1: winter-ready chassis

Build or adapt the rolling platform first.

The goal is:

- stable frame
- proper wheelbase and track width
- strong tool mounting area
- room for engine, battery, electronics, and guards

Do not start with the auger housing if the base platform is still weak.

### Stage 2: engine module

Mount the donor mower engine as a separate power module.

Focus on:

- rigid mounting points
- vibration isolation where appropriate
- throttle control linkage
- kill-switch integration
- exhaust and heat routing
- fuel access and shielding

### Stage 3: auger transmission

Build the mechanical path from engine to auger.

Focus on:

- pulley or sprocket ratios
- tensioning
- guarded transmission path
- overload protection
- safe disengagement

### Stage 4: electric traction

Once the machine can survive mechanically, finalize the electrical drive.

Focus on:

- left and right motor packaging
- controller cooling
- cable routing
- connector sealing
- battery restraint and enclosure

### Stage 5: instrumentation and ROS integration

At this stage, the repository becomes directly useful for the real machine.

Add telemetry for:

- battery voltage
- battery current
- battery temperature
- traction current
- engine RPM
- auger RPM
- auger engaged state
- auger overload state
- chute position
- deflector position

## Recommended First Sensors

You do not need full autonomy first.
You need visibility into whether the machine is healthy.

Best first sensors:

- hall or encoder feedback for traction speed
- RPM pickup for engine
- RPM pickup for auger shaft
- current sensors for traction bus
- battery voltage sensor
- battery temperature sensor
- limit switches for chute and deflector actuators

## What The Current Repo Already Covers

The current repository already gives you:

- keyboard operator control
- work modes
- emergency stop state
- snow mode behavior shaping
- a hybrid command layer for engine, auger, chute, and deflector control
- simulated hybrid telemetry for battery, DC bus, engine RPM, and auger RPM

That means the repo is already useful as the control-system skeleton.
The next step is to move it from a simulated hybrid controller toward real
hardware interfaces.

## What Should Change In Software Next

The repository now already exposes a simulated hybrid state model with:

- `engine_enabled`
- `engine_throttle`
- `auger_engaged`
- `chute_position`
- `deflector_position`
- `simulate_auger_overload`
- `/hybrid/engine_rpm`
- `/hybrid/battery_soc`
- `/hybrid/battery_voltage`
- `/hybrid/dc_bus_current`
- `/hybrid/traction_power_limit`
- `/hybrid/auger_rpm`
- `/hybrid/auger_overload`
- `/hybrid/auger_interlock_ok`
- `/hybrid/auger_fault_latched`
- `/hybrid/auger_reset_required`
- `/hybrid/auger_status_text`

The next software job is no longer inventing these interfaces.
The next software job is wiring them to real IO:

- engine RPM pickup
- traction current measurement
- battery monitor
- auger clutch actuator
- chute rotation actuator
- deflector actuator
- real overload detection instead of manual simulation
- physical jam reset and clutch-disengage semantics

The software goal stays the same: represent the real machine honestly, not as a
single normalized tool slider.

## First Purchase Priorities

If budget is limited, buy in this order:

1. mechanical protection for the auger
2. safe chassis and mounting materials
3. traction-side motor controllers and wiring
4. battery and BMS
5. sensors and actuators for telemetry and control

If the auger protection and chassis are weak, the rest of the project becomes
expensive noise.

## Decision Summary

The most realistic first serious version is:

- electric traction platform
- combustion engine as the main power source
- mechanical auger drive
- ROS 2 handling operator input, state, safety, and telemetry

That is modern enough to scale, cheap enough to start, and honest about where
the real engineering risk actually lives.
