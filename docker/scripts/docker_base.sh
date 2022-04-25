#!/usr/bin/env bash

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# source "${TOP_DIR}/scripts/apollo.bashrc"

unset TOP_DIR

export HOST_ARCH="$(uname -m)"
export HOST_OS="$(uname -s)"

DOCKER_RUN_CMD="docker run"


function remove_container_if_exists() {
    local container="$1"
    if docker ps -a --format '{{.Names}}' | grep -q "${container}"; then
        info "Removing existing Ltslam container: ${container}"
        docker stop "${container}" >/dev/null
        docker rm -v -f "${container}" 2>/dev/null
    fi
}

function postrun_start_user() {
    local container="$1"
    if [ "${USER}" != "root" ]; then
        docker exec -u root "${container}" \
            bash -c '/apollo/scripts/docker_start_user.sh'
    fi
}

function stop_all_ltslam_containers() {
    local force="$1"
    local running_containers
    running_containers="$(docker ps -a --format '{{.Names}}')"
    for container in ${running_containers[*]}; do
        if [[ "${container}" =~ apollo_.*_${USER} ]]; then
            #printf %-*s 70 "Now stop container: ${container} ..."
            #printf "\033[32m[DONE]\033[0m\n"
            #printf "\033[31m[FAILED]\033[0m\n"
            info "Now stop container ${container} ..."
            if docker stop "${container}" >/dev/null; then
                if [[ "${force}" == "-f" || "${force}" == "--force" ]]; then
                    docker rm -f "${container}" 2>/dev/null
                fi
                info "Done."
            else
                warning "Failed."
            fi
        fi
    done
}


export -f stop_all_ltslam_containers remove_container_if_exists
export DOCKER_RUN_CMD