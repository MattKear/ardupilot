# syntax=docker/dockerfile:1
FROM public.ecr.aws/docker/library/alpine:latest as FS_ARDU
# Trick to copy the source code into a directory and remove docker things from it to have faster rebuild

COPY . /ardupilot
RUN rm -rf /ardupilot/ardupilot_docker

FROM public.ecr.aws/docker/library/python:3.10-slim AS builder
# cannot use python 3.11 with copter 4.2

# Make a working directory
WORKDIR /ardupilot

################################################################################
### Install minimal build tools and remove cache. Don't do any update

RUN apt-get update && apt-get install --no-install-recommends -y \
    g++ \
    git \
    libxml2-dev \
    libxslt-dev \
    ssh \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
ENV DISABLE_MAVNATIVE=True
RUN python3 -m pip install --user --no-deps --no-cache-dir future pymavlink empy pexpect ptyprocess

COPY --from=FS_ARDU /ardupilot /ardupilot
# RUN /ardupilot/Tools/gittools/submodule-sync.sh

COPY ardupilot_docker/MNA383.json /ardupilot/Tools/autotest/models/MNA383.json

# Build binary
RUN ./waf distclean \
    && ./waf configure --board sitl --disable-scripting --no-submodule-update \
    && ./waf copter

# Second stage build
FROM public.ecr.aws/docker/library/python:3.11-slim AS runner

WORKDIR /ardupilot

ENV DISABLE_MAVNATIVE=True
# copy only the dependencies installation from the 1st stage image
COPY --from=builder /root/.local /root/.local

# Copy binary and defaut param file from previous image
COPY --from=builder /ardupilot/build/sitl/bin/arducopter .
COPY --from=builder /ardupilot/Tools/autotest/default_params/copter.parm .
COPY --from=builder /ardupilot/Tools/autotest/default_params/copter-octaquad.parm .
COPY ardupilot_docker/sitl_launch.bash /ardupilot/sitl_launch.bash
COPY ardupilot_docker/383.parm /ardupilot/383.parm
COPY ardupilot_docker/ccdl.parm /ardupilot/ccdl.parm
COPY ardupilot_docker/no_ccdl.parm /ardupilot/no_ccdl.parm
COPY ardupilot_docker/terrain /ardupilot/terrain
COPY ardupilot_docker/logs /ardupilot/logs

# Docker's supervisor
COPY ardupilot_docker/supervisord.conf /etc/supervisor/supervisord.conf

ENV PATH=/root/.local/bin:$PATH

RUN python3 -m pip install supervisor --no-cache-dir \
    && mkdir -p /ardupilot/fcu1 \
    && mkdir -p /ardupilot/fcu2 \
    && mkdir -p /ardupilot/fcu3 \
    && cp -r /ardupilot/terrain /ardupilot/fcu1 \
    && cp -r /ardupilot/terrain /ardupilot/fcu2 \
    && cp -r /ardupilot/terrain /ardupilot/fcu3 \
    && cp -r /ardupilot/logs /ardupilot/fcu1 \
    && cp -r /ardupilot/logs /ardupilot/fcu2 \
    && cp -r /ardupilot/logs /ardupilot/fcu3

################################################################################
#### Setup the entrypoint == application that will launch by default

# SITL 1
EXPOSE 5760/tcp
EXPOSE 5762/tcp
EXPOSE 5763/tcp
EXPOSE 5501
# SITL 2
EXPOSE 5770/tcp
EXPOSE 5772/tcp
EXPOSE 5773/tcp
EXPOSE 5511
# SITL 3
EXPOSE 5780/tcp
EXPOSE 5782/tcp
EXPOSE 5783/tcp
EXPOSE 5521

# Multicast
EXPOSE 14550/tcp
EXPOSE 14550/udp

# JSON PORTS
# EXPOSE 9012/tcp
# EXPOSE 9022/tcp

ARG SITL_LAT="53.346248"
ARG SITL_LON="-7.715564"
ARG SITL_ALT="64.8"
ARG SITL_HEADING="0"
ENV SITL_LAT=${SITL_LAT}
ENV SITL_LON=${SITL_LON}
ENV SITL_ALT=${SITL_ALT}
ENV SITL_HEADING=${SITL_HEADING}

# Expose Port (Note: if you change it do it as well in surpervisord.conf)
EXPOSE 9001
HEALTHCHECK --interval=15s CMD supervisorctl status | grep -q "RUNNING" || exit 1
CMD ["/usr/local/bin/supervisord"]
