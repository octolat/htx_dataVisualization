FROM python:3.14-slim

RUN apt-get update && apt-get install -y python3-pip libgtk-3-dev libxkbcommon-x11-0 vulkan-tools

RUN apt-get -y install \
    libclang-dev \
    libatk-bridge2.0 \
    libfontconfig1-dev \
    libfreetype6-dev \
    libglib2.0-dev \
    libgtk-3-dev \
    libssl-dev \
    libxcb-render0-dev \
    libxcb-shape0-dev \
    libxcb-xfixes0-dev \
    libxkbcommon-dev \
    patchelf

WORKDIR /src

COPY ./visualisation/requirements.txt ./

RUN python3 -m pip install --no-cache-dir -r requirements.txt

COPY ./visualisation ./visualisation

WORKDIR /src/visualisation
CMD ["python", "repl.py"]
