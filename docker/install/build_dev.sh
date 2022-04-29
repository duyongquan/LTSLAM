#!/usr/bin/env bash

DOCKERFILE=$1

CONTEXT="$(dirname "${BASH_SOURCE[0]}")"

REPO=xslam/ltslam
TAG="${REPO}"

# Fail on first error.
set -e
docker build -t ${TAG} -f ${DOCKERFILE} ${CONTEXT}
echo "Built new image ${TAG}"