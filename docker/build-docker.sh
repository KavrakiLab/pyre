#!/bin/bash

cd "${0%/*}"
docker build -t pyre -f Dockerfile `git rev-parse --show-toplevel`

