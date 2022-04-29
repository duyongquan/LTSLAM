# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

wget https://apollocache.blob.core.windows.net/apollo-docker/ota_security.tar.gz
tar xzf ota_security.tar.gz
pushd ota_security
bash ota_server_deploy.sh root
popd

# Clean up.
rm -fr ota_security.tar.gz ota_security