# syntax=docker/dockerfile:1
FROM agt-yhs-base:v1
SHELL ["/bin/bash", "-c"]
ARG NAVIGATION_COMMIT=unknown
ARG HMI_PATCH_SHA256=unknown
# One source/build layer avoids redundant full filesystem snapshots on VFS hosts.
# Contexts are independent pinned checkouts; no repository is vendored into another.
RUN --mount=type=bind,source=.,target=/run/navigation,ro \
    --mount=type=bind,from=mapping,source=.,target=/run/mapping,ro \
    --mount=type=bind,from=hmi,source=.,target=/run/hmi,ro \
    --mount=type=secret,id=proxy_ca \
    if [ -f /run/secrets/proxy_ca ]; then cp /run/secrets/proxy_ca /usr/local/share/ca-certificates/session-proxy.crt; update-ca-certificates; fi; \
    mkdir -p /opt/nav_ws/src/agt_navigation_v3 /opt/mapping_ws/src/agt-lio-pgo-mapping /opt/hmi /opt/agt && \
    cp -a /run/navigation/. /opt/nav_ws/src/agt_navigation_v3/ && \
    cp -a /run/mapping/. /opt/mapping_ws/src/agt-lio-pgo-mapping/ && \
    cp -a /run/hmi/. /opt/hmi/ && \
    cp /opt/nav_ws/src/agt_navigation_v3/appliance/repos.lock.yaml /opt/agt/versions.yaml && \
    printf 'navigation_commit: %s\nhmi_patch_sha256: %s\n' "$NAVIGATION_COMMIT" "$HMI_PATCH_SHA256" > /opt/agt/build_commits.yaml && \
    python3 /opt/nav_ws/src/agt_navigation_v3/appliance/scripts/build_sources.py && \
    bash /opt/nav_ws/src/agt_navigation_v3/appliance/scripts/build_runtime.sh && \
    python3 -m pip install --no-deps /opt/nav_ws/src/agt_navigation_v3/appliance && \
    rm -f /usr/local/share/ca-certificates/session-proxy.crt && update-ca-certificates
ENV LD_LIBRARY_PATH=/usr/local/lib
WORKDIR /data
ENTRYPOINT ["/opt/nav_ws/src/agt_navigation_v3/appliance/scripts/entrypoint.sh"]
