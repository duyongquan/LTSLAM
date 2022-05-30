#!/usr/bin/env bash

DOCKERFILE=$1

CONTEXT="$(dirname "${BASH_SOURCE[0]}")"

# copy ltslam docker
if [ -d LTSLAM ]; then
    rm -rf ./LTSLAM
fi
mkdir LTSLAM
cp -r ${CONTEXT}/../* ./LTSLAM

REPO=xslam/ltslam
TAG="${REPO}"

# Fail on first error.
set -e
docker build -t ${TAG} -f ${DOCKERFILE} ${CONTEXT}
echo "Built new image ${TAG}"
rm -rf ./LTSLAM