ARG IDF_VERSION=release-v6.0
FROM docker.io/espressif/idf:${IDF_VERSION}
RUN echo "source /opt/esp/idf/export.sh" >> /root/.bashrc
# Fix esp-idf menuconfig locale setting
RUN echo "export LANG=C" >> /root/.bashrc
RUN echo "export LC_ALL=C" >> /root/.bashrc

RUN apt-get update && apt-get install -y \
    udev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

ARG CLANGD_VERSION=19.1.2
RUN curl -L https://github.com/clangd/clangd/releases/download/${CLANGD_VERSION}/clangd-linux-${CLANGD_VERSION}.zip -o /tmp/clangd.zip \
    && unzip /tmp/clangd.zip -d /opt \
    && ln -s /opt/clangd_${CLANGD_VERSION}/bin/clangd /usr/local/bin/clangd \
    && rm /tmp/clangd.zip