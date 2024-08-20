ARG WAVEMAP_TAG=main

FROM alpine:3.20

ARG WAVEMAP_TAG

# hadolint ignore=DL3018
RUN apk add --no-cache git build-base python3-dev py3-pip

# hadolint ignore=DL3059
RUN git clone --branch ${WAVEMAP_TAG} https://github.com/ethz-asl/wavemap.git

WORKDIR /wavemap/library/python
# hadolint ignore=DL3042
RUN pip3 install . --break-system-packages
