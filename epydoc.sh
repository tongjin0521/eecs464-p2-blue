#!/bin/bash
mkdir -p doc/html
epydoc \
  --inheritance listed \
  --show-private \
  -o doc/html \
  --graph all \
  --html \
  py/ckbot/*.py py/*.py py/joy/*.py demos/*.py

