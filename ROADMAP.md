# MiniBot Roadmap

This file provides an overview of the direction this project is heading, broken out by each sub-project

## [Complete PCBs](https://github.com/DDeGonge/MiniBot/tree/main/pcbs)

This is the primary focus point due to long lead times. Estimated completion for all boards is July 6, 2026

### MiniBot Mainboard (Revision)
- Fix stepper motor header
- Replace individual red leds with dual color led, red for charge and white is mcu controlled
- Add contact pads for pogo pin charging (tbd)

### Server Motherboard (Initial)
- Designed to use ESP32-DevkitC-32E Dev Board on headers
- 20 h-bridge drivers for 20 electromagnets embedded in board. Shared direction pin
- 1 h-bridge driver split to drive 2x solenoids for clock paddle automation
- GPIO expander to talk with 16 of the electromagnet h-bridges
- 2x inputs for clock paddle switches

### Charge Case Board (Initial)
- 6x2 array of charging slots
- Designed to use either usb c ports OR pogo pins. Will be testing both configurations

## Bugfixing New Bot Features

A number of new features were introduced a little haphazardly and are not yet working as intended. This includes:

- Changes to the stepper motion control to free up cpu during "coast" period. Plan to use hardware timer.
- Changes to the mag sensor collection loop to also use hardware timer to reduce missed reads and also free up cpu
- Implementation of new controllable white led, with patterns for different operating mode and error conditions

## Create Board-Specific UI

The current UI is more of a debug interface optimized for large screens. Create a user-focused UI for a 4" touchscreen
that will ideally use all of the same backend functionality as the existing debug UI.

- Modularize any existing code as needed for the debug UI
- New frontend optimized for touch controls, keep other debug frontend for testing on PC
- Integrate new panel for piece connection status and battery level
- New "gameplay" panels for playing puzzles, AI, or other humans

## Improve Path Planning

Path planning makes use of a conflict based approach with some additional routines to get "unstuck". But piece still do
get stuck sometimes, and the approach is far from optimal.

- Modify unstuck routines to improve reliability
- Identify stuck cases earlier and shift to an unstick method
- Optimize path planning to run more moves in parallel if the pieces don't interact
- Testing with real board would probably be smart.
- Generalized graveyard positions, allow pieces to navigate to closest one that wont block other positions

## Implement Chess Engine

Pretty self-explanatory. Need a game state class to keep track of the gameplay, determine if moves are legal, and so on

- Create the game engine class
- Hook existing piece objects into it
- Connect game engine to the appropriate UI page for gameplay

## Integrate Chess Puzzles/AI

No plan for this yet, but will need to do something for games against AI and setting up chess puzzles. Former can probably
be a locally run chess bot of user configurable ELO. Latter may need an API hookup or just a large library of stored puzzles.

- Figure this part out

## Power Optimizations

Once all the functionality is in place (or sooner, idk), something needs to be done to improve battery life of the bots.

- Allow auto light sleep by implementing windowed communication
- Potential pcb changes to reduce stepper motor power draw
- Intelligently adjust position estimation update rate when pieces are static
