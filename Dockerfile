# Use an existing image as the base
FROM ros:noetic-robot

# Set the working directory inside the container
WORKDIR /setup

# Install dependencies (if applicable)
RUN apt-get update && apt-get install -y \
    python3-pip \
    qt5-qmake \
    qtbase5-dev \
    qtdeclarative5-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install sip before installing PyQt5
RUN pip install sip

COPY ./requirements.txt /setup
RUN pip install -r requirements.txt

COPY ./entrypoint.sh /setup
RUN chmod +x /setup/entrypoint.sh
RUN chown root:root /setup/entrypoint.sh

# Set the entrypoint to the correct path
ENTRYPOINT ["/setup/entrypoint.sh"]

WORKDIR /med_mon

CMD ["bash"]