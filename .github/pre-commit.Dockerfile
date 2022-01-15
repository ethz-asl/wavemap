FROM ubuntu:focal

RUN apt-get update && \
    apt-get install -y git python3-pip clang-format cppcheck && \
    pip3 install pre-commit cpplint && \
    rm -rf /var/lib/apt/lists/*
