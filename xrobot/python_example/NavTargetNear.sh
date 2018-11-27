#!/bin/bash
echo "XWorld3DNavTargetNear"

PYTHONPATH=..:$PYTHONPATH __GL_SYNC_TO_VBLANK=0 python XWorld3DNavTargetNear_TaskGroup.py
