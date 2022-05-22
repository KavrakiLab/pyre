#!/bin/bash

cd "${0%/*}"
docker build -t pyre . `git rev-parse --show-toplevel`

