# rcx-printer

This repository contains code for a dot matrix printer built using Lego Mindstorms RCX.

`printer` contains code for the RCX brick.
It requires `brickOS` (available as package for Debian/Ubuntu).

`img2rcx` is a software to transmit images to the RCX.
Since it has very little available memory, images are transmitted in blockwise.
`img2rcx` acts as a server from which the RCX can request the image's blocks via IR.