ARG IDF_VERSION=release-v5.5
FROM docker.io/espressif/idf:${IDF_VERSION}

RUN apt-get update && apt-get install -y \
    zsh \
    curl \
    udev \
    usbutils \
    sudo \
    && rm -rf /var/lib/apt/lists/*
RUN curl -sS https://starship.rs/install.sh | sh -s -- -y


ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN existing_user=$(getent passwd $USER_UID | cut -d: -f1) && \
    if [ -n "$existing_user" ]; then \
        usermod -l $USERNAME -d /home/$USERNAME -m -s /bin/zsh $existing_user && \
        groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1) 2>/dev/null || true; \
    else \
        groupadd --gid $USER_GID $USERNAME && \
        useradd --uid $USER_UID --gid $USER_GID -m -s /bin/zsh $USERNAME; \
    fi && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

RUN echo 'source /opt/esp/idf/export.sh' >> /home/$USERNAME/.zshrc && \
    echo 'export LANG=C' >> /home/$USERNAME/.zshrc && \
    echo 'export LC_ALL=C' >> /home/$USERNAME/.zshrc && \
    echo 'eval "$(starship init zsh)"' >> /home/$USERNAME/.zshrc

RUN mkdir -p /home/$USERNAME/workspace
WORKDIR /home/$USERNAME/workspace

CMD ["/bin/zsh"]