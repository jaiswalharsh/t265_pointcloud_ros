#!/bin/bash

export DOCKER_BUILDKIT=1
docker build --network=host -t t265_pointcloud:inspectrone -f Dockerfile .
