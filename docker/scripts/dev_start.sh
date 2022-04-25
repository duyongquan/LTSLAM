#!/usr/bin/env bash

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${CURR_DIR}/docker_base.sh"

CACHE_ROOT_DIR="${LTSLAM_ROOT_DIR}/.cache"

DOCKER_REPO="ltslam/ltslam"
DEV_CONTAINER="ltslam_dev_${USER}"
DEV_INSIDE="in-dev-docker"

SUPPORTED_ARCHS=(x86_64)
TARGET_ARCH="$(uname -m)"

VERSION_X86_64="dev-x86_64-18.04"

FAST_MODE="no"
USE_LOCAL_IMAGE=0

VOLUME_VERSION="latest"
SHM_SIZE="2G"
USER_SPECIFIED_MAPS=
MAP_VOLUMES_CONF=
OTHER_VOLUMES_CONF=

DEFAULT_MAPS=(
    sunnyvale_big_loop
    sunnyvale_loop
    sunnyvale_with_two_offices
    san_mateo
)

DEFAULT_TEST_MAPS=(
    sunnyvale_big_loop
    sunnyvale_loop
)

function show_usage() {
    cat <<EOF
Usage: $0 [options] ...
OPTIONS:
    -h, --help             Display this help and exit.
    -f, --fast             Fast mode without pulling all map volumes.
    -l, --local            Use local docker image.
    -t, --tag <TAG>        Specify docker image with tag <TAG> to start.
    --shm-size <bytes>     Size of /dev/shm . Passed directly to "docker run"
    stop                   Stop all running Ltslam containers.
EOF
}

function parse_arguments() {
    local custom_version=""
    local custom_dist=""
    local shm_size=""

    while [ $# -gt 0 ]; do
        local opt="$1"
        shift
        case "${opt}" in
            -t | --tag)
                if [ -n "${custom_version}" ]; then
                    warning "Multiple option ${opt} specified, only the last one will take effect."
                fi
                custom_version="$1"
                shift
                optarg_check_for_opt "${opt}" "${custom_version}"
                ;;

            -h | --help)
                show_usage
                exit 1
                ;;

            -f | --fast)
                FAST_MODE="yes"
                ;;

            -l | --local)
                USE_LOCAL_IMAGE=1
                ;;

            --shm-size)
                shm_size="$1"
                shift
                optarg_check_for_opt "${opt}" "${shm_size}"
                ;;

            stop)
                info "Now, stop all Ltslam containers created by ${USER} ..."
                stop_all_ltslam_containers "-f"
                exit 0
                ;;
            *)
                warning "Unknown option: ${opt}"
                exit 2
                ;;
        esac
    done # End while loop

    [[ -n "${shm_size}" ]] && SHM_SIZE="${shm_size}"
}

function determine_dev_image() {
    local version="$1"
    # If no custom version specified
    if [[ -z "${version}" ]]; then
        if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
                version="${VERSION_X86_64}"
        else
            error "Logic can't reach here! Please report this issue to Ltslam@GitHub."
            exit 3
        fi
    fi
    DEV_IMAGE="${DOCKER_REPO}:${version}"
}

function check_host_environment() {
    if [[ "${HOST_OS}" != "Linux" ]]; then
        warning "Running Ltslam dev container on ${HOST_OS} is UNTESTED, exiting..."
        exit 1
    fi
}

function check_target_arch() {
    local arch="${TARGET_ARCH}"
    for ent in "${SUPPORTED_ARCHS[@]}"; do
        if [[ "${ent}" == "${TARGET_ARCH}" ]]; then
            return 0
        fi
    done
    error "Unsupported target architecture: ${TARGET_ARCH}."
    exit 1
}

function setup_devices_and_mount_local_volumes() {
    local __retval="$1"

    [ -d "${CACHE_ROOT_DIR}" ] || mkdir -p "${CACHE_ROOT_DIR}"

    source "${LTSLAM_ROOT_DIR}/scripts/ltslam_base.sh"
    setup_device

    local volumes="-v $LTSLAM_ROOT_DIR:/ltslam"
    local teleop="${LTSLAM_ROOT_DIR}/../ltslam-teleop"
    if [ -d "${teleop}" ]; then
        volumes="-v ${teleop}:/ltslam/modules/teleop ${volumes}"
    fi

    local os_release="$(lsb_release -rs)"
    case "${os_release}" in
        16.04)
            warning "[Deprecated] Support for Ubuntu 16.04 will be removed" \
                "in the near future. Please upgrade to ubuntu 18.04+."
            volumes="${volumes} -v /dev:/dev"
            ;;
        18.04 | 20.04 | *)
            volumes="${volumes} -v /dev:/dev"
            ;;
    esac
    # local tegra_dir="/usr/lib/aarch64-linux-gnu/tegra"
    # if [[ "${TARGET_ARCH}" == "aarch64" && -d "${tegra_dir}" ]]; then
    #    volumes="${volumes} -v ${tegra_dir}:${tegra_dir}:ro"
    # fi
    volumes="${volumes} -v /media:/media \
                        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                        -v /etc/localtime:/etc/localtime:ro \
                        -v /usr/src:/usr/src \
                        -v /lib/modules:/lib/modules"
    volumes="$(tr -s " " <<<"${volumes}")"
    eval "${__retval}='${volumes}'"
}

function docker_pull() {
    local img="$1"
    if [[ "${USE_LOCAL_IMAGE}" -gt 0 ]]; then
        if docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "${img}"; then
            info "Local image ${img} found and will be used."
            return
        fi
        warning "Image ${img} not found locally although local mode enabled. Trying to pull from remote registry."
    fi
    if [[ -n "${GEO_REGISTRY}" ]]; then
        img="${GEO_REGISTRY}/${img}"
    fi

    info "Start pulling docker image ${img} ..."
    if ! docker pull "${img}"; then
        error "Failed to pull docker image : ${img}"
        exit 1
    fi
}

function docker_restart_volume() {
    local volume="$1"
    local image="$2"
    local path="$3"
    info "Create volume ${volume} from image: ${image}"
    docker_pull "${image}"
    docker volume rm "${volume}" >/dev/null 2>&1
    docker run -v "${volume}":"${path}" --rm "${image}" true
}

function restart_map_volume_if_needed() {
    local map_name="$1"
    local map_version="$2"
    local map_volume="ltslam_map_volume-${map_name}_${USER}"
    local map_path="/ltslam/modules/map/data/${map_name}"

    if [[ ${MAP_VOLUMES_CONF} == *"${map_volume}"* ]]; then
        info "Map ${map_name} has already been included."
    else
        local map_image=
        if [ "${TARGET_ARCH}" = "aarch64" ]; then
            map_image="${DOCKER_REPO}:map_volume-${map_name}-${TARGET_ARCH}-${map_version}"
        else
            map_image="${DOCKER_REPO}:map_volume-${map_name}-${map_version}"
        fi
        info "Load map ${map_name} from image: ${map_image}"

        docker_restart_volume "${map_volume}" "${map_image}" "${map_path}"
        MAP_VOLUMES_CONF="${MAP_VOLUMES_CONF} --volume ${map_volume}:${map_path}"
    fi
}

function mount_map_volumes() {
    info "Starting mounting map volumes ..."
    if [ -n "${USER_SPECIFIED_MAPS}" ]; then
        for map_name in ${USER_SPECIFIED_MAPS}; do
            restart_map_volume_if_needed "${map_name}" "${VOLUME_VERSION}"
        done
    fi

    if [[ "$FAST_MODE" == "no" ]]; then
        for map_name in ${DEFAULT_MAPS[@]}; do
            restart_map_volume_if_needed "${map_name}" "${VOLUME_VERSION}"
        done
    else
        for map_name in ${DEFAULT_TEST_MAPS[@]}; do
            restart_map_volume_if_needed "${map_name}" "${VOLUME_VERSION}"
        done
    fi
}

function main() {
    check_host_environment
    check_target_arch

    parse_arguments "$@"

    if [[ "${USE_LOCAL_IMAGE}" -gt 0 ]]; then
        info "Start docker container based on local image : ${DEV_IMAGE}"
    fi

    # if ! docker_pull "${DEV_IMAGE}"; then
    #     error "Failed to pull docker image ${DEV_IMAGE}"
    #     exit 1
    # fi

    info "Remove existing Ltslam Development container ..."
    remove_container_if_exists ${DEV_CONTAINER}

    # local local_volumes= 
    # setup_devices_and_mount_local_volumes local_volumes

    # mount_map_volumes

    info "Starting Docker container \"${DEV_CONTAINER}\" ..."

    local local_host="$(hostname)"
    local display="${DISPLAY:-:0}"
    local user="${USER}"
    local uid="$(id -u)"
    local group="$(id -g -n)"
    local gid="$(id -g)"

    set -x

    # ${DOCKER_RUN_CMD} -itd \
    #     --privileged \
    #     --name "${DEV_CONTAINER}" \
    #     -e DISPLAY="${display}" \
    #     -e DOCKER_USER="${user}" \
    #     -e USER="${user}" \
    #     -e DOCKER_USER_ID="${uid}" \
    #     -e DOCKER_GRP="${group}" \
    #     -e DOCKER_GRP_ID="${gid}" \
    #     -e DOCKER_IMG="${DEV_IMAGE}" \
    #     ${MAP_VOLUMES_CONF} \
    #     ${local_volumes} \
    #     --net host \
    #     -w /ltslam \
    #     --add-host "${DEV_INSIDE}:127.0.0.1" \
    #     --add-host "${local_host}:127.0.0.1" \
    #     --hostname "${DEV_INSIDE}" \
    #     --shm-size "${SHM_SIZE}" \
    #     --pid=host \
    #     -v /dev/null:/dev/raw1394 \
    #     "${DEV_IMAGE}" \
    #     /bin/bash

    # if [ $? -ne 0 ]; then
    #     error "Failed to start docker container \"${DEV_CONTAINER}\" based on image: ${DEV_IMAGE}"
    #     exit 1
    # fi
    # set +x

    # postrun_start_user "${DEV_CONTAINER}"

    # ok "Congratulations! You have successfully finished setting up LTSLAM Dev Environment."
    # ok "To login into the newly created ${DEV_CONTAINER} container, please run the following command:"
    # ok "  bash docker/scripts/dev_into.sh"
    # ok "Enjoy!"
}

main "$@"