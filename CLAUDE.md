# CLAUDE.md

# Project Overview

## Project Name

3-DOF Robotic Arm Digital Twin & Physical Robot

## Vision

Develop a professional-quality robotics software framework that begins as a complete digital twin and culminates in a fully functional 3D printed robotic arm.

This is **not** simply a robot simulator.

The goal is to create a reusable robotics platform demonstrating:

* Modern C++ software engineering
* Robotics mathematics
* Forward and inverse kinematics
* Motion planning
* Simulation
* CAD automation
* Embedded firmware
* Mechanical design
* Digital twin architecture

The physical robot should ultimately become another backend of the software, rather than requiring any changes to the core algorithms.

---

# Guiding Principles

Always prioritize:

1. Correctness
2. Modularity
3. Extensibility
4. Readability
5. Physical realism

Never optimize prematurely.

Never tightly couple visualization, simulation, CAD, or hardware.

Every subsystem should be independently testable.

---

# Development Philosophy

Simulation First.

Nothing should be printed or purchased until the simulator demonstrates that the design works.

Every mechanical parameter should first exist digitally.

The CAD model should validate the simulation.

The physical robot should validate the CAD.

---

# Final Hardware Goals

Desktop robotic manipulator.

Configuration:

* 3 DOF
* Base rotation
* Shoulder pitch
* Elbow pitch
* End-effector gripper

Actuation:

* NEMA 17 stepper motors
* TMC2209 drivers
* Planetary or cycloidal reductions
* Internal bearings supporting all structural loads
* Gearboxes transmit torque only

Printed on:

Bambu Lab A1 Mini

Maximum printable dimensions:

180 mm × 180 mm × 180 mm

---

# Software Stack

## Core Language

C++20

Use modern C++ idioms:

* RAII
* Smart pointers
* constexpr where appropriate
* const correctness
* strong typing
* STL algorithms
* templates only when beneficial

Avoid:

* global variables
* raw owning pointers
* duplicated logic

---

## Python

Used for:

* validation
* plotting
* optimization
* CAD scripting
* analysis

Not for production simulation.

---

## Frontend

React

Tailwind CSS

React Three Fiber

Three.js

---

## Compilation

CMake

Target platforms:

* Windows
* Linux
* WebAssembly

---

# Repository Structure

src/

* math/
* geometry/
* kinematics/
* dynamics/
* planning/
* simulation/
* visualization/
* hardware/
* io/
* app/

tests/

assets/

config/

docs/

cad/

python/

---

# Configuration

Robot parameters must never be hardcoded.

Store all robot parameters inside YAML.

Example:

* link lengths
* masses
* inertias
* joint limits
* gearbox ratios
* motor specifications
* bearing dimensions
* workspace limits

Changing the robot should require editing configuration files instead of source code.

---

# Kinematics

The robot consists of

Joint 1

Base rotation

θ₁

Joint 2

Shoulder pitch

θ₂

Joint 3

Elbow pitch

θ₃

The core library shall support:

Forward Kinematics

Inverse Kinematics

Jacobian computation

Workspace analysis

Joint limit enforcement

Reachability analysis

---

## Inverse Kinematics

Use an analytical geometric solution.

Avoid Denavit-Hartenberg matrices unless later expansion requires them.

Requirements:

* O(1) solution
* elbow-up
* elbow-down
* unreachable target handling
* singularity detection
* configurable joint limits

---

# Dynamics

The simulator should evolve beyond simple kinematics.

Eventually include:

Mass matrix

Gravity

Center of mass

Inertia tensors

Motor torque limits

Gearbox efficiency

Joint friction

Payload effects

Dynamic equations should eventually support inverse dynamics.

---

# Motion Planning

Support:

Point-to-point motion

Linear interpolation

Cubic splines

Quintic splines

Velocity limits

Acceleration limits

Jerk limits

Future:

RRT

RRT*

Obstacle avoidance

---

# Digital Twin

The simulator is the primary product.

The hardware is simply another implementation.

The simulator should contain:

Robot model

World

Collision objects

Physics

Joint limits

Workspace visualization

Trajectory visualization

Interactive controls

Performance metrics

---

# Visualization

Rendering must remain independent from robot mathematics.

Renderer responsibilities:

Display robot

Display coordinate frames

Display joint axes

Display target

Display workspace

Display singularities

Display trajectories

No kinematic calculations should occur inside rendering code.

---

# Hardware Abstraction

Create an interface:

IRobot

SimulationRobot

HardwareRobot

Both implementations should expose identical APIs.

The remainder of the application should not know whether it is communicating with simulation or physical hardware.

---

# CAD Integration

CAD is downstream of simulation.

Fusion 360 should generate:

Links

Joint housings

Bearing pockets

Gearboxes

Motor mounts

Gripper

Cable routing

Fastener locations

All CAD dimensions should originate from configuration files where practical.

---

# Mechanical Design

Design around standard components.

Examples:

608 bearings

8 mm shafts

NEMA 17 motors

Heat-set inserts

Metric fasteners

Do not design custom dimensions when industry-standard hardware exists.

---

# Gearboxes

The simulator must support gearbox parameters.

Include:

Reduction ratio

Efficiency

Backlash

Rotor inertia

Maximum torque

These values should affect the simulation.

Support both:

Planetary gearboxes

Cycloidal gearboxes

---

# Motor Model

Stepper motors should not be treated as ideal actuators.

Include:

Maximum speed

Continuous torque

Holding torque

Acceleration limits

Microstepping resolution

Skipped-step detection

---

# Testing

Every mathematical function requires unit tests.

Test:

Forward kinematics

Inverse kinematics

Round-trip FK → IK → FK consistency

Joint limits

Workspace boundaries

Singularities

Edge cases

Regression tests

---

# Documentation

Every subsystem must include:

Overview

Architecture

Algorithms

Equations

Assumptions

Limitations

Future improvements

Use Markdown.

---

# Coding Standards

Prefer:

Small classes

Single responsibility

Descriptive names

Clear APIs

Comments explaining why, not what

Avoid:

Magic numbers

Duplicate code

Overly clever abstractions

---

# Development Roadmap

## Phase 1

Core mathematics

Forward kinematics

Inverse kinematics

Configuration system

Unit tests

## Phase 2

Trajectory planning

Simulation engine

Python validation

Workspace visualization

## Phase 3

React + WebAssembly integration

Interactive digital twin

Real-time visualization

## Phase 4

Fusion 360 CAD generation

Gearbox integration

Mechanical validation

Mass properties

## Phase 5

Embedded firmware

ESP32 or STM32

Stepper control

Serial communication

Limit switches

Homing

## Phase 6

Physical robot

Assembly

Calibration

Motion validation

Performance tuning

---

# Definition of Success

The project is considered complete when:

* The digital twin accurately models the physical robot.
* The same C++ library powers simulation, WebAssembly, and embedded firmware.
* CAD is generated from shared configuration.
* The robot can execute smooth trajectories within joint limits.
* All mathematical components are unit tested.
* Documentation is sufficient for another engineer to understand and extend the project.
* The resulting repository demonstrates professional software engineering, robotics, and mechanical design suitable for a high-quality engineering portfolio.