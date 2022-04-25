DOCKERFILE=$1

CONTEXT="$(dirname "${BASH_SOURCE[0]}")"

REPO=LTSLAM/ltslam
ARCH=$(uname -m)
TIME=$(date +%Y%m%d_%H%M)

TAG="${REPO}:dev-18.04-${ARCH}-${TIME}"

# Fail on first error.
set -e
docker build -t ${TAG} -f ${DOCKERFILE} ${CONTEXT}
echo "Built new image ${TAG}"