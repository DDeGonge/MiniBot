# MiniBot Roadmap

This file provides an overview of the direction this project is heading, broken out by each sub-project
Last updated September 8, 2026

## CAD (https://cad.onshape.com/documents/4f8eaef75458146767928ab5/w/f159d6d65b9091531e1ead34/e/13b51722310f27710438c727?renderMode=0&uiState=6aa0a6ac3ed7120f46e7fd94)

Short term
- Finish clock
- Finish board
- Finish toppers

Next rev
- Strengthen topper retainer tabs (requires pcb outline changes)
- Figure out some way for more repeatable wheel protrusion, make it adjustable somehow?

## PCBs (https://github.com/DDeGonge/MiniBot/tree/main/pcbs)

### MiniBot Mainboard
- Fix the charge status led polarity, oops
- Move pogo pin contacts to board edge
- Remove old pogo pin pads

- No changes planned.

### Charge Case Board
- Finalize pogo pin charge layout and method, big TBD

## MiniBot Firmware

- Fine tune all the newly implemented changes
- Confirm light sleep is working
- Intelligently enable and disable position sensing
- Improve motion planning, switch to discrete linear, arc, and rotate in place commands

## Server Firmware

- Improve reliability of the UART coms with app layer
- Figure out what else needs fixing


## Application

### Create Board-Specific UI

The current UI is more of a debug interface optimized for large screens. Create a user-focused UI for a 4" touchscreen
that will ideally use all of the same backend functionality as the existing debug UI.

- Modularize any existing code as needed for the debug UI
- New frontend optimized for touch controls, keep other debug frontend for testing on PC
- Integrate new panel for piece connection status and battery level
- New "gameplay" panels for playing puzzles, AI, or other humans

### Improve Path Planning

Path planning makes use of a conflict based approach with some additional routines to get "unstuck". But piece still do
get stuck sometimes, and the approach is far from optimal.

- Modify unstuck routines to improve reliability
- Identify stuck cases earlier and shift to an unstick method
- Optimize path planning to run more moves in parallel if the pieces don't interact
- Testing with real board would probably be smart.
- Generalized graveyard positions, allow pieces to navigate to closest one that wont block other positions

### Implement Chess Engine

Pretty self-explanatory. Need a game state class to keep track of the gameplay, determine if moves are legal, and so on

- Create the game engine class
- Hook existing piece objects into it
- Connect game engine to the appropriate UI page for gameplay

### Integrate Chess Puzzles/AI

No plan for this yet, but will need to do something for games against AI and setting up chess puzzles. Former can probably
be a locally run chess bot of user configurable ELO. Latter may need an API hookup or just a large library of stored puzzles.

- Figure this part out
