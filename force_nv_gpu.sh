#!/bin/bash
# Run render_app via this script when it picks integrated GPU over NVidia (requires NVidia Prime)
DRI_PRIME=1 ./render_app $*
