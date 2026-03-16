#!/usr/bin/env bash

## Install TypeDB (keyring-based apt source, jammy distro)
RUN mkdir -p /etc/apt/keyrings \
    && gpg --batch --keyserver hkps://keyserver.ubuntu.com --recv-key 17507562824CFDCC \
    && gpg --batch --export 17507562824CFDCC > /etc/apt/keyrings/typedb.gpg \
    && chmod 644 /etc/apt/keyrings/typedb.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/typedb.gpg] https://repo.typedb.com/public/public-release/deb/ubuntu jammy main" \
    > /etc/apt/sources.list.d/typedb.list

RUN apt-get update && apt-get install -y \
    typedb \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install typedb-driver
