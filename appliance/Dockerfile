# syntax=docker/dockerfile:1
FROM agt-yhs-base:v1 AS native-runtime
SHELL ["/bin/bash", "-c"]
# Independent pinned source contexts; immutable native layer reused by GUI changes.
RUN --mount=type=bind,source=.,target=/run/navigation,ro \
    --mount=type=bind,from=mapping,source=.,target=/run/mapping,ro \
    --mount=type=secret,id=proxy_ca \
    if [ -f /run/secrets/proxy_ca ]; then cp /run/secrets/proxy_ca /usr/local/share/ca-certificates/session-proxy.crt; update-ca-certificates; fi; \
    mkdir -p /opt/nav_ws/src/agt_navigation_v3 /opt/mapping_ws/src/agt-lio-pgo-mapping /opt/agt && \
    cp -a /run/navigation/. /opt/nav_ws/src/agt_navigation_v3/ && \
    cp -a /run/mapping/. /opt/mapping_ws/src/agt-lio-pgo-mapping/ && \
    cp /opt/nav_ws/src/agt_navigation_v3/appliance/repos.lock.yaml /opt/agt/versions.yaml && \
    python3 /opt/nav_ws/src/agt_navigation_v3/appliance/scripts/build_sources.py && \
    AGT_SKIP_HMI=1 bash /opt/nav_ws/src/agt_navigation_v3/appliance/scripts/build_runtime.sh && \
    python3 -m pip install --no-deps /opt/nav_ws/src/agt_navigation_v3/appliance && \
    rm -f /usr/local/share/ca-certificates/session-proxy.crt && update-ca-certificates
FROM native-runtime
RUN --mount=type=bind,from=hmi,source=.,target=/run/hmi,ro \
    --mount=type=secret,id=proxy_ca \
    if [ -f /run/secrets/proxy_ca ]; then cp /run/secrets/proxy_ca /usr/local/share/ca-certificates/session-proxy.crt; update-ca-certificates; fi; \
    mkdir -p /opt/hmi && cp -a /run/hmi/. /opt/hmi/ && \
    bash /opt/nav_ws/src/agt_navigation_v3/appliance/scripts/build_hmi.sh && \
    rm -f /usr/local/share/ca-certificates/session-proxy.crt && update-ca-certificates
ARG NAVIGATION_COMMIT=unknown
ARG HMI_PATCH_SHA256=unknown
ENV AGT_NAVIGATION_COMMIT=$NAVIGATION_COMMIT AGT_HMI_PATCH_SHA256=$HMI_PATCH_SHA256 LD_LIBRARY_PATH=/usr/local/lib
WORKDIR /
ENTRYPOINT ["/opt/nav_ws/src/agt_navigation_v3/appliance/scripts/entrypoint.sh"]
