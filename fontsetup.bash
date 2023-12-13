#!/usr/bin/bash

# The font used for the Speedometer.
# Any font from the DSEG font family (v0.46) can be used.
# https://www.keshikan.net/fonts-e.html
DSEG_FONT=./fonts/DSEG14Classic-Italic.ttf

# The font used thoughout the rest of the UI.
DEFAULT_FONT=./fonts/Roboto-Regular.ttf

cp $DSEG_FONT ../../dseg.ttf
cp $DEFAULT_FONT ../../default.ttf