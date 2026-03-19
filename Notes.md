# 🤖 Robot Configuration & CAN Map

## 🏗️ Subsystem Hardware Specs

### Shooter
* **Flywheels:** 2x (X44) — *Mechanically Aligned/Linked*
* **Hood:** 1x (X44)
* **Hood Encoder** (Ctre Through Bore)
* **Turret:** 1x (X44)
* **Turret Encoder** (Ctre Through Bore)

### Intake
* **Rollers:** 1x (X60) — *Front/Top/Bottom connected*
* **Wrist:** 1x (X60)

### Indexer
* **Top Roller:** 1x (X60) — *Independent control for logic*
* **Belt:** 1x (X60)
* **Agitator** 1x (X60)
* **Kicker** 1x (X60)

### Climb (Not yet finished)
* **Lift:** 2x (X60)
* **Ratchet:** 1x servo?

---

## ⚡ CAN ID Assignments

### Drivetrain (Swerve) | Range: 1-13
| Device | ID | Notes |
| :--- | :--- | :--- |
| **Pigeon 2.0** | 1 | Gyro |
| **Front Left Drive** | 2 | |
| **Front Left Steer** | 3 | |
| **FL Encoder** | 4 | |
| **Front Right Drive**| 5 | |
| **Front Right Steer**| 6 | |
| **FR Encoder** | 7 | |
| **Back Right Drive** | 8 | |
| **Back Right Steer** | 9 | |
| **BR Encoder** | 10 | |
| **Back Left Drive** | 11 | |
| **Back Left Steer** | 12 | |
| **BL Encoder** | 13 | |

### Shooter | Range: 20-24
| Device | ID | Notes |
| :--- | :--- | :--- |
| **Main Flywheel** | 20 | Leader |
| **Follower Flywheel**| 21 | Follows 20 |
| **Hood Motor** | 22 | |
| **Hood Encoder** | 23 | |
| **Turret Motor** | 24 | |
| **Turret Encoder** | 25 | |

### Intake | Range: 30-32
| Device | ID | Notes |
| :--- | :--- | :--- |
| **Intake Wrist** | 30 | Flex X60 |
| **Main Roller** | 31 | Leader |

### Indexer | Range: 40-43
| Device | ID | Notes |
| :--- | :--- | :--- |
| **Top Roller**| 40 | |
| **Conveyor** | 41 | |
| **Agitator** | 42 | |
| **Kicker** | 43 | |

### Climb | Range: 50-51 (Not yet finished)
| Device | ID | Notes |
| :--- | :--- | :--- |
| **Climb Left** | 50 | |
| **Climb Right** | 51 | |
| **Climb Servo** | idk yet | |

---

## 🎮 Controller Bindings (To Be Defined)

* left middle bumper = left dpad
* right middle bumper = right dpad
* Back right top button = up dpad
* Back right bottom button = down dpad

### Driver
| Trigger | Action |
| :--- | :--- |
|**Left Stick:** | Translation |
|**Right Stick:** | Rotation |
|**Dpad Down:** | Reset Odom/Tare |
|**Left Trigger:** | Intake |
|**Left Top Bumper:** | Outtake |
|**Left Middle Bumper:** | Spit |
|**Right Trigger:** | Shoot |
|**Right Top Bumper:** | Agitate/Fold Intake |
|**Right Middle Bumper:** | -- |
|**Back Right Top Button:** | Trench |
|**Back Right Bottom Button:** | Non Trench |


### Operator
| Trigger | Action |
| :--- | :--- |
| **Left Trigger:** | Shooter Focus Auto|
| **Left Middle Bumper:** | --|
| **Right Trigger:** | Agitate/Fold Intake|
| **Right Top Bumper:** | --|
| **Right Middle Bumper:** | --|
| **Back Right Top Button:** | --|
| **Back Right Bottom Button:** | --|
| **X Button:** | Climb Align Left|
| **Y Button:** | Climb |
| **A Button:** | Cancel Climb|
| **B Button:** | Climb Align Right|
