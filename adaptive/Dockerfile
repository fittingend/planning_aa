# syntax=docker/dockerfile:1.4
# Build with --build-context para_sdk=<local PARA SDK directory>.
# Keep this image local unless its SDK distribution terms are settled.
FROM ubuntu:22.04 AS build
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates cmake ninja-build g++ libeigen3-dev libboost-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=para_sdk / /opt/para-sdk
COPY . /src
RUN cmake -S /src/adaptive -B /build -G Ninja \
      -DPARA_SDK=/opt/para-sdk \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/planning-ap \
    && cmake --build /build --parallel 4 \
    && cmake --install /build --component Standalone \
    && test -x /opt/planning-ap/bin/EM \
    && test -x /opt/planning-ap/opt/PlanningAA/bin/PlanningAA

FROM ubuntu:22.04 AS runtime
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    libstdc++6 libgcc-s1 ca-certificates \
    && rm -rf /var/lib/apt/lists/*
ARG PARA_SDK_VERSION=unknown
LABEL para.sdk.version="${PARA_SDK_VERSION}"
COPY --from=build /opt/planning-ap /opt/planning-ap
COPY adaptive/entrypoint.sh /opt/planning-ap/entrypoint.sh
WORKDIR /opt/planning-ap
ENTRYPOINT ["/opt/planning-ap/entrypoint.sh"]
