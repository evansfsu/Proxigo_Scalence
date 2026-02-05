# Proxigo Scalence - PX4 SITL Container
# Pre-built PX4 SITL for faster startup
#
# Build: docker build -t proxigo/px4-sitl:latest -f docker/px4_sitl/Dockerfile .
# Run: docker run --rm -it --network host proxigo/px4-sitl:latest

FROM px4io/px4-dev-simulation-jammy:latest

# Set environment
ENV PX4_HOME=/root/PX4-Autopilot
ENV PATH="${PX4_HOME}/build/px4_sitl_default/bin:${PATH}"

# Clone and build PX4 (this takes time but only happens during build)
WORKDIR /root
RUN git clone --depth 1 --branch v1.14.3 https://github.com/PX4/PX4-Autopilot.git && \
    cd PX4-Autopilot && \
    git submodule update --init --recursive --depth 1

# Build PX4 SITL
WORKDIR /root/PX4-Autopilot
RUN make px4_sitl_default

# Set default parameters for vision-based navigation
RUN echo "param set-default EKF2_EV_CTRL 15" >> /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS
RUN echo "param set-default EKF2_HGT_REF 3" >> /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS
RUN echo "param set-default EKF2_GPS_CTRL 0" >> /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS
RUN echo "param set-default COM_ARM_WO_GPS 1" >> /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS

# Default home location (Delaware test site)
ENV PX4_HOME_LAT=39.678123
ENV PX4_HOME_LON=-75.750456
ENV PX4_HOME_ALT=50.0

# Create startup script
RUN echo '#!/bin/bash\n\
cd /root/PX4-Autopilot\n\
export PX4_SIM_MODEL=${PX4_SIM_MODEL:-iris}\n\
export PX4_SIM_HOST_ADDR=${PX4_SIM_HOST_ADDR:-localhost}\n\
\n\
echo "Starting PX4 SITL..."\n\
echo "Model: $PX4_SIM_MODEL"\n\
echo "Home: $PX4_HOME_LAT, $PX4_HOME_LON, $PX4_HOME_ALT"\n\
echo "MAVLink: UDP port 14540"\n\
echo ""\n\
\n\
# Start without simulator (headless mode for MAVROS)\n\
make px4_sitl_default none_$PX4_SIM_MODEL\n\
' > /root/start_sitl.sh && chmod +x /root/start_sitl.sh

WORKDIR /root/PX4-Autopilot

# Expose MAVLink ports
EXPOSE 14540/udp
EXPOSE 14550/udp
EXPOSE 14560/udp

# Default command
CMD ["/root/start_sitl.sh"]
