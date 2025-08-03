ARG PYTHON_VERSION="3.10"
ARG DEBIAN_VERSION="bookworm"

FROM python:${PYTHON_VERSION}-slim-${DEBIAN_VERSION} AS base

ARG USER=user
ARG HOME=/app
ARG UID=1000
ARG GID=1000
RUN \
  addgroup --gid "${GID}" "${USER}" \
  && adduser --disabled-password --gecos "" --home "${HOME}" --ingroup "${USER}" --uid "${UID}" "${USER}"

ARG DEBIAN_VERSION="bookworm"
ARG SYSTEM_PACKAGES=""
RUN \
  ([ -n "${SYSTEM_PACKAGES}" ] \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install --yes --no-install-recommends $(printf "${SYSTEM_PACKAGES}" | xargs) \
  && apt-get autoremove -yqq --purge \
  && apt-get clean \
  || true) \
  && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /usr/share/man /usr/share/doc /usr/share/doc-base

ARG PYTHON_MODULE=""
RUN \
  printf '#!/bin/sh\n\n' > /entrypoint.sh \
  && ([ -z "${PYTHON_MODULE}" ] && printf 'exec "$@"\n' || printf "exec python -m ${PYTHON_MODULE}\n") >> /entrypoint.sh \
  && chmod 0755 /entrypoint.sh

ARG PYTHON_PACKAGE_EXTRAS=""
WORKDIR /build
COPY dist/*.whl dist/
COPY dist/vendor vendor
RUN \
  su "${USER}" -c '\
  extras=$(printf "${PYTHON_PACKAGE_EXTRAS}" | sed -e "s/[[:space:]]\+/,/g") \
  && package=$(find dist -maxdepth 1 -iname "*.whl" -exec realpath {} \;) \
  && package=$([ -z "${extras}" ] && printf "${package}" || printf "${package} [${extras}]") \
  && python -m pip install \
  --isolated \
  --user \
  --no-index \
  --no-color \
  --no-cache-dir \
  --disable-pip-version-check \
  --no-python-version-warning \
  --no-warn-script-location \
  --progress-bar off \
  --find-links "file://$(realpath vendor)" \
  "${package}" \
  && python -m compileall -q ${HOME}' \
  && cd / \
  && rm -rf /build

USER ${USER}
WORKDIR ${HOME}
ENV PATH="${HOME}/.local/bin:${PATH}"
ENTRYPOINT ["/entrypoint.sh"]
