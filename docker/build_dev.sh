#!/usr/bin/env bash

DOCKERFILE=$1

CONTEXT="$(dirname "${BASH_SOURCE[0]}")"

# copy 3rdpary docker
cp -r ${CONTEXT}/../3rdparty .

REPO=xslam/ltslam
TAG="${REPO}"

# Fail on first error.
set -e
docker build -t ${TAG} -f ${DOCKERFILE} ${CONTEXT}
echo "Built new image ${TAG}"
rm -rf 3rdparty