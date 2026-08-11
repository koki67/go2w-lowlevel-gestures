# syntax=docker/dockerfile:1.7

FROM ubuntu:22.04 AS builder

ARG DEBIAN_FRONTEND=noninteractive
ARG CYCLONEDDS_COMMIT=1be07de395e4ddf969db2b90328cdf4fb73e9a64
ARG UNITREE_SDK2_PYTHON_COMMIT=a035adeaa6f8ea171bef9a43e8477abb87a0b35e

ENV CYCLONEDDS_HOME=/opt/cyclonedds \
    PATH=/opt/venv/bin:${PATH} \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONPATH=/opt/unitree_sdk2_python

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        bison \
        build-essential \
        ca-certificates \
        cmake \
        flex \
        git \
        python3-dev \
        python3-pip \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN git init /src/cyclonedds \
    && git -C /src/cyclonedds remote add origin https://github.com/eclipse-cyclonedds/cyclonedds.git \
    && git -C /src/cyclonedds fetch --depth 1 origin "${CYCLONEDDS_COMMIT}" \
    && git -C /src/cyclonedds checkout --detach FETCH_HEAD \
    && test "$(git -C /src/cyclonedds rev-parse HEAD)" = "${CYCLONEDDS_COMMIT}"

RUN cmake -S /src/cyclonedds -B /src/cyclonedds/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${CYCLONEDDS_HOME}" \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TESTING=OFF \
    && cmake --build /src/cyclonedds/build --parallel \
    && cmake --install /src/cyclonedds/build

RUN git init /src/unitree_sdk2_python \
    && git -C /src/unitree_sdk2_python remote add origin https://github.com/unitreerobotics/unitree_sdk2_python.git \
    && git -C /src/unitree_sdk2_python fetch --depth 1 origin "${UNITREE_SDK2_PYTHON_COMMIT}" \
    && git -C /src/unitree_sdk2_python checkout --detach FETCH_HEAD \
    && test "$(git -C /src/unitree_sdk2_python rev-parse HEAD)" = "${UNITREE_SDK2_PYTHON_COMMIT}"

# SDK2Py's b2 hierarchy intentionally has no __init__.py files.  Its upstream
# setup.py uses find_packages(), so a wheel silently omits that hierarchy and
# the package's own top-level import then fails.  Run the verified checkout as
# source, matching Unitree's documented source-tree usage, without patching it.
RUN install -d /opt/unitree_sdk2_python \
    && cp -a /src/unitree_sdk2_python/unitree_sdk2py /opt/unitree_sdk2_python/ \
    && cp /src/unitree_sdk2_python/LICENSE /opt/unitree_sdk2_python/LICENSE

RUN python3 -m venv /opt/venv \
    && python -m pip install \
        pip==24.0 \
        setuptools==70.3.0 \
        wheel==0.43.0 \
    && python -m pip install \
        cyclonedds==0.10.2 \
        numpy==1.26.4 \
        opencv-python==4.10.0.84 \
    && python -m pip check

WORKDIR /app
COPY go2w_gesture_real.py /app/go2w_gesture_real.py
COPY go2w_gesture_real_legacy.py /app/go2w_gesture_real_legacy.py
COPY tests /app/tests

RUN python -m unittest discover -s /app/tests -v \
    && python /app/go2w_gesture_real.py --describe \
    && python /app/go2w_gesture_real_legacy.py --describe


FROM ubuntu:22.04 AS runtime

ARG DEBIAN_FRONTEND=noninteractive
ARG CYCLONEDDS_COMMIT=1be07de395e4ddf969db2b90328cdf4fb73e9a64
ARG UNITREE_SDK2_PYTHON_COMMIT=a035adeaa6f8ea171bef9a43e8477abb87a0b35e

LABEL org.opencontainers.image.title="Go2W low-level gestures" \
      org.opencontainers.image.description="Fail-closed Unitree Go2W hardware gestures using SDK2Py" \
      org.opencontainers.image.source="https://github.com/koki67/go2w-lowlevel-gestures" \
      org.opencontainers.image.licenses="MIT" \
      io.unitree.sdk2-python.commit="${UNITREE_SDK2_PYTHON_COMMIT}" \
      io.cyclonedds.commit="${CYCLONEDDS_COMMIT}"

ENV CYCLONEDDS_HOME=/opt/cyclonedds \
    LD_LIBRARY_PATH=/opt/cyclonedds/lib \
    PATH=/opt/venv/bin:${PATH} \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONPATH=/opt/unitree_sdk2_python \
    PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        libgl1 \
        libglib2.0-0 \
        libice6 \
        libsm6 \
        libxext6 \
        libxrender1 \
        python3 \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --uid 1000 --shell /bin/bash controller

COPY --from=builder /opt/cyclonedds /opt/cyclonedds
COPY --from=builder /opt/unitree_sdk2_python /opt/unitree_sdk2_python
COPY --from=builder /opt/venv /opt/venv
COPY --from=builder --chown=controller:controller /app /app

USER controller
WORKDIR /app

ENTRYPOINT ["/opt/venv/bin/python", "/app/go2w_gesture_real.py"]
CMD []
