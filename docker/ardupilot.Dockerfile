# ArduSub SITL. The builder stage needs the full ArduPilot toolchain; the
# runtime stage keeps only what `sim_vehicle.py -N` uses: the ardusub binary,
# Tools/autotest (sim_vehicle.py, pysim, default params) and MAVProxy.
#
# QGC connectivity: SITL exposes MAVLink on tcp 5760 (primary) and udp 14550
# (QGC default). The JSON bridge to the Gazebo ArduPilotPlugin is on
# 127.0.0.1:9002 (--model JSON:127.0.0.1). When run via docker-compose.yml the
# command is overridden to include --out=udp:0.0.0.0:14550 so QGC on the host
# (or any device on the same network) can connect without extra flags.
# See compose file for the full bringup command including --custom-location
# fix for the depth-hold baro/altitude bug.
FROM ardupilot/ardupilot-dev-base:edge AS build

RUN apt-get update && apt-get install --no-install-recommends -y git \
  && git config --global url."https://github.com/".insteadOf git://github.com/

RUN git clone --depth=1 https://github.com/ArduPilot/ardupilot.git /ardupilot
WORKDIR /ardupilot
RUN git submodule update --init --recursive --depth=1

# Enable ArduPilot's object-avoidance path planner (OA_TYPE: BendyRuler /
# Dijkstra) and the proximity library (PRX*_TYPE, e.g. MAVLink
# OBSTACLE_DISTANCE from a depth camera) in ArduSub. Both exist in the
# libraries and are wired up in ArduCopter, but ArduSub never instantiates
# the planner or initialises/updates proximity. Also: a GUIDED position target
# without a yaw follows WP_YAW_BEHAVIOR (e.g. face along the path) instead of
# freezing the current heading. Written against master @ 30a9288; if a newer
# master no longer applies, regenerate the patch.
COPY misc/patches/ardupilot/ardusub_object_avoidance.patch /tmp/ardusub_object_avoidance.patch
RUN git apply /tmp/ardusub_object_avoidance.patch

# Only the vehicle the simulator flies.
RUN ./waf configure --board sitl && ./waf sub

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3 python3-venv procps tzdata \
  && rm -rf /var/lib/apt/lists/*

# MAVProxy fans SITL out to the --out ports and provides --console. Its
# declared dependencies include VTK, OpenCV and matplotlib (~1 GB) for the
# map/graph GUI modules, which SITL never loads; install the core only.
RUN python3 -m venv /opt/mavenv \
  && /opt/mavenv/bin/pip install --no-cache-dir --no-deps MAVProxy \
  && /opt/mavenv/bin/pip install --no-cache-dir pymavlink pyserial numpy defusedxml pynmeagps pexpect
ENV PATH="/opt/mavenv/bin:${PATH}"

COPY --from=build /ardupilot/build/sitl/bin/ardusub /ardupilot/build/sitl/bin/ardusub
COPY --from=build /ardupilot/Tools/autotest /ardupilot/Tools/autotest
# sim_vehicle.py checks the vehicle source directory exists.
COPY --from=build /ardupilot/ArduSub /ardupilot/ArduSub
WORKDIR /ardupilot

# MAVLink for QGC / companion computer; JSON FDM bridge to Gazebo.
EXPOSE 5760/tcp
EXPOSE 5762/tcp
EXPOSE 14550/udp
EXPOSE 14555/udp
EXPOSE 9002/tcp
EXPOSE 9002/udp

# Default command is overridden by docker-compose.yml (which adds --custom-location,
# --add-param-file and --out for QGC). This ENTRYPOINT remains usable for
# standalone `docker run`:
#   docker run --network host ghcr.io/davidnoronha1/mira_sim-ardupilot-sitl --out=udp:0.0.0.0:14550
ENTRYPOINT ["/ardupilot/Tools/autotest/sim_vehicle.py", "-N", "-f", "vectored_6dof", "-v", "ArduSub", "--console", "--model", "JSON:127.0.0.1"]
