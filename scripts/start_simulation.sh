#!/bin/bash
#
# Proxigo UAV Simulation Launcher
# 
# Cross-platform script that works on:
#   - Linux (native)
#   - Windows (via WSL2)
#   - Nvidia Orin Nano
#
# Usage:
#   ./scripts/start_simulation.sh [OPTIONS]
#
# Options:
#   --vehicle TYPE    Vehicle: advanced_plane (default), standard_vtol, x500
#   --location NAME   Location preset: death_valley (default), zurich, custom
#   --lat LAT         Custom latitude
#   --lon LON         Custom longitude
#   --headless        Run without GUI
#   --stop            Stop simulation
#   --restart         Restart simulation
#   --status          Show status
#
# Examples:
#   ./scripts/start_simulation.sh                          # Start with defaults
#   ./scripts/start_simulation.sh --vehicle standard_vtol  # VTOL aircraft
#   ./scripts/start_simulation.sh --restart                # Restart simulation

set -e

# ============================================================================
# Configuration
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="$PROJECT_DIR/docker/simulation/docker-compose.simulation.yml"

# Default settings
VEHICLE="advanced_plane"
LOCATION="death_valley"
HEADLESS=false
ACTION="start"

# Location presets
declare -A LOCATIONS
LOCATIONS["death_valley"]="36.2329,-116.8276,0"
LOCATIONS["zurich"]="47.3977,8.5456,488"
LOCATIONS["austin"]="30.2672,-97.7431,150"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ============================================================================
# Functions
# ============================================================================

log_info() {
    echo -e "${CYAN}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_banner() {
    echo -e "${CYAN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║           PROXIGO UAV SIMULATION                             ║"
    echo "║           Fixed-Wing & VTOL Development Platform             ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

detect_platform() {
    if grep -qi microsoft /proc/version 2>/dev/null; then
        PLATFORM="wsl"
        log_info "Platform: Windows (WSL2)"
    elif [[ -f /etc/nv_tegra_release ]]; then
        PLATFORM="orin"
        log_info "Platform: Nvidia Orin"
    else
        PLATFORM="linux"
        log_info "Platform: Linux"
    fi
}

setup_display() {
    if [[ "$HEADLESS" == "true" ]]; then
        export DISPLAY=""
        export LIBGL_ALWAYS_SOFTWARE=1
        log_info "Running in headless mode"
        return
    fi

    case "$PLATFORM" in
        wsl)
            # WSL2: Use host Windows display
            if [[ -z "$DISPLAY" ]]; then
                # Try to get Windows host IP
                WIN_HOST=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
                export DISPLAY="${WIN_HOST}:0"
            fi
            log_info "Display: $DISPLAY (Windows host)"
            
            # Check if X server is accessible
            if ! timeout 2 bash -c "echo > /dev/tcp/${WIN_HOST}/6000" 2>/dev/null; then
                log_warn "X server not detected on Windows"
                log_warn "Please start VcXsrv/XLaunch with 'Disable access control' checked"
            fi
            ;;
        linux|orin)
            if [[ -z "$DISPLAY" ]]; then
                export DISPLAY=:0
            fi
            # Allow X connections
            xhost +local:docker 2>/dev/null || true
            log_info "Display: $DISPLAY"
            ;;
    esac
}

check_docker() {
    if ! command -v docker &> /dev/null; then
        log_error "Docker not found. Please install Docker."
        exit 1
    fi

    if ! docker info &> /dev/null; then
        log_error "Docker daemon not running. Please start Docker."
        exit 1
    fi

    log_success "Docker is running"
}

check_image() {
    if ! docker image inspect proxigo/px4-gazebo:harmonic &> /dev/null; then
        log_warn "PX4-Gazebo image not found. Building..."
        docker build -t proxigo/px4-gazebo:harmonic -f "$PROJECT_DIR/docker/px4_sitl/Dockerfile.gazebo" "$PROJECT_DIR"
    else
        log_success "PX4-Gazebo image found"
    fi
}

parse_location() {
    if [[ -n "${LOCATIONS[$LOCATION]}" ]]; then
        IFS=',' read -r PX4_HOME_LAT PX4_HOME_LON PX4_HOME_ALT <<< "${LOCATIONS[$LOCATION]}"
    fi
    
    export PX4_HOME_LAT
    export PX4_HOME_LON
    export PX4_HOME_ALT
    export PX4_VEHICLE="$VEHICLE"
    
    log_info "Location: $LOCATION ($PX4_HOME_LAT, $PX4_HOME_LON, alt=${PX4_HOME_ALT}m)"
    log_info "Vehicle: $VEHICLE"
}

stop_simulation() {
    log_info "Stopping simulation..."
    docker compose -f "$COMPOSE_FILE" down --remove-orphans 2>/dev/null || true
    docker rm -f px4_sitl mavlink_router 2>/dev/null || true
    log_success "Simulation stopped"
}

start_simulation() {
    log_info "Starting simulation..."
    
    # Export environment
    export DISPLAY
    export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-0}
    
    # Start services
    docker compose -f "$COMPOSE_FILE" up -d
    
    log_info "Waiting for PX4 to initialize (this may take 2-3 minutes on first run)..."
    
    # Wait for PX4 to be ready
    local timeout=300
    local elapsed=0
    while [[ $elapsed -lt $timeout ]]; do
        if docker logs px4_sitl 2>&1 | grep -q "pxh>"; then
            log_success "PX4 SITL is ready!"
            break
        fi
        
        # Show build progress
        local progress=$(docker logs px4_sitl 2>&1 | grep -oE '\[[0-9]+/[0-9]+\]' | tail -1)
        if [[ -n "$progress" ]]; then
            echo -ne "\r  Building: $progress ($elapsed sec)    "
        fi
        
        sleep 5
        elapsed=$((elapsed + 5))
    done
    echo ""
    
    if [[ $elapsed -ge $timeout ]]; then
        log_warn "Timeout waiting for PX4. Checking status..."
        docker logs px4_sitl 2>&1 | tail -20
    fi
}

set_ekf_origin() {
    log_info "Setting EKF origin to simulation location..."
    
    python3 - << EOF
import sys
import time
try:
    from pymavlink import mavutil
    
    # Try different connection methods
    for conn in ['tcp:127.0.0.1:5760', 'udp:127.0.0.1:14550']:
        try:
            mav = mavutil.mavlink_connection(conn, timeout=5)
            mav.wait_heartbeat(timeout=10)
            
            lat = int($PX4_HOME_LAT * 1e7)
            lon = int($PX4_HOME_LON * 1e7)
            alt = int($PX4_HOME_ALT * 1000)
            
            mav.mav.set_gps_global_origin_send(mav.target_system, lat, lon, alt)
            time.sleep(0.5)
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                179, 0, 0, 0, 0, 0, $PX4_HOME_LAT, $PX4_HOME_LON, $PX4_HOME_ALT
            )
            print(f"EKF origin set via {conn}")
            sys.exit(0)
        except Exception as e:
            continue
    
    print("Could not set EKF origin - set manually in QGC")
except ImportError:
    print("pymavlink not installed - install with: pip3 install pymavlink")
EOF
}

upload_mission() {
    log_info "Uploading default mission..."
    python3 "$PROJECT_DIR/scripts/upload_death_valley_mission.py" 2>/dev/null || \
        log_warn "Mission upload failed - upload manually via QGC"
}

show_status() {
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  SIMULATION STATUS${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
    echo ""
    
    # Check containers
    echo "Containers:"
    docker ps --filter "name=px4" --filter "name=mavlink" --format "  {{.Names}}: {{.Status}}" 2>/dev/null || echo "  No containers running"
    
    echo ""
    echo "Location:"
    echo "  Latitude:  ${PX4_HOME_LAT:-36.2329} N"
    echo "  Longitude: ${PX4_HOME_LON:--116.8276} W"
    echo "  Altitude:  ${PX4_HOME_ALT:-0} m"
    
    echo ""
    echo "QGroundControl Connection:"
    echo "  UDP: udp://127.0.0.1:14550 (auto-detect)"
    echo "  TCP: tcp://127.0.0.1:5760 (manual)"
    
    echo ""
    echo "Commands:"
    echo "  Restart:  $0 --restart"
    echo "  Stop:     $0 --stop"
    echo "  Logs:     docker logs -f px4_sitl"
    echo ""
}

print_usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Options:
  --vehicle TYPE    Vehicle type: advanced_plane, standard_vtol, x500
  --location NAME   Location: death_valley, zurich, austin, or custom
  --lat LAT         Custom latitude (requires --location custom)
  --lon LON         Custom longitude
  --alt ALT         Custom altitude
  --headless        Run without GUI
  --stop            Stop simulation
  --restart         Restart simulation  
  --status          Show status only
  --help            Show this help

Examples:
  $0                                    # Start Death Valley fixed-wing
  $0 --vehicle standard_vtol            # Start VTOL
  $0 --location zurich                  # Start at Zurich
  $0 --lat 36.0 --lon -117.0 --alt 100  # Custom location
  $0 --restart                          # Restart simulation
EOF
}

# ============================================================================
# Main
# ============================================================================

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --vehicle)
            VEHICLE="$2"
            shift 2
            ;;
        --location)
            LOCATION="$2"
            shift 2
            ;;
        --lat)
            PX4_HOME_LAT="$2"
            LOCATION="custom"
            shift 2
            ;;
        --lon)
            PX4_HOME_LON="$2"
            shift 2
            ;;
        --alt)
            PX4_HOME_ALT="$2"
            shift 2
            ;;
        --headless)
            HEADLESS=true
            shift
            ;;
        --stop)
            ACTION="stop"
            shift
            ;;
        --restart)
            ACTION="restart"
            shift
            ;;
        --status)
            ACTION="status"
            shift
            ;;
        --help|-h)
            print_usage
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

# Execute
print_banner
detect_platform
check_docker

case "$ACTION" in
    stop)
        stop_simulation
        ;;
    restart)
        stop_simulation
        sleep 2
        check_image
        parse_location
        setup_display
        start_simulation
        set_ekf_origin
        show_status
        ;;
    status)
        parse_location
        show_status
        ;;
    start)
        check_image
        parse_location
        setup_display
        start_simulation
        set_ekf_origin
        upload_mission
        show_status
        ;;
esac
