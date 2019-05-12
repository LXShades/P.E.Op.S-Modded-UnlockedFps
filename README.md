# P.E.Op.S-Modded-UnlockedFps
A mod of Pete's OpenGL plugins for PlayStation emulators that creates high frame rates with interpolation. Currently in prototyping phase.

# Overview
This is an early prototype of a vertex interpolator for the P.E.Op.S OpenGL GPU. The goal is to achieve the appearance of a fully unlocked frame rate in the majority of PS1 games.

The process works a little like this:
* Records draw calls for the current frame
* On frame display:
* * Try to "match" previous polygons with current polygons
* * Redraw the polygons with vertices interpolated with the previous frame by an increasing blend factor, until the expected original frame duration has passed
* * Swap draw calls into 'previous frame'
* * Empty the current frame draw calls; rinse and repeat

The project is currently in the prototyping phase, and as such should not be expected to be clean or functional!

# Compiling
* Project file uses Visual Studio 2017 (C++14/C++17)
* Debugging properties and post-build events have been modified to work with ePSXe on my PC. Feel free to disable these in your own version