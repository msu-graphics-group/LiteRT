#!/bin/bash

# List of scenes
scenes=("chair" "drums" "ficus" "hotdog" "lego" "materials" "mic" "ship")

# Loop through each scene
for scene in "${scenes[@]}"; do
    # Loop through numbers from 00000 to 00299
    for i in $(seq -f "%05g" 0 299); do
        # Launch the render_app with the scene and number
        ./render_app "$scene" "$i"
    done
done
