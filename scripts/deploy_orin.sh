#!/bin/bash
# Proxigo Scalence - Deploy to Orin Nano
#
# Usage:
#   ./scripts/deploy_orin.sh                    # Deploy to orin.local
#   ./scripts/deploy_orin.sh 192.168.1.100      # Deploy to specific IP
#   ./scripts/deploy_orin.sh orin.local nvidia  # Deploy with custom user

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

ORIN_HOST=${1:-"orin.local"}
ORIN_USER=${2:-"nvidia"}
DEPLOY_PATH=${3:-"~/proxigo"}

echo "=============================================="
echo "Proxigo Scalence - Orin Nano Deployment"
echo "=============================================="
echo "Target: $ORIN_USER@$ORIN_HOST"
echo "Deploy Path: $DEPLOY_PATH"
echo "=============================================="

# Test SSH connection
echo ""
echo ">>> Testing SSH connection..."
if ! ssh -o ConnectTimeout=5 "$ORIN_USER@$ORIN_HOST" "echo 'Connection successful'"; then
    echo "ERROR: Cannot connect to $ORIN_USER@$ORIN_HOST"
    echo "Ensure SSH is enabled and the Orin is accessible"
    exit 1
fi

# Create deployment directory
echo ""
echo ">>> Creating deployment directory..."
ssh "$ORIN_USER@$ORIN_HOST" "mkdir -p $DEPLOY_PATH/{config,satellite_data,logs,scripts}"

# Sync configuration files
echo ""
echo ">>> Syncing configuration..."
rsync -avz --delete \
    "$PROJECT_DIR/config/" \
    "$ORIN_USER@$ORIN_HOST:$DEPLOY_PATH/config/"

# Sync Docker Compose files
echo ""
echo ">>> Syncing Docker Compose files..."
rsync -avz \
    "$PROJECT_DIR/docker-compose.yml" \
    "$PROJECT_DIR/docker-compose.dev.yml" \
    "$ORIN_USER@$ORIN_HOST:$DEPLOY_PATH/"

# Sync satellite data (if exists and not empty)
if [ -d "$PROJECT_DIR/satellite_data" ] && [ "$(ls -A "$PROJECT_DIR/satellite_data" 2>/dev/null)" ]; then
    echo ""
    echo ">>> Syncing satellite data..."
    rsync -avz --delete \
        "$PROJECT_DIR/satellite_data/" \
        "$ORIN_USER@$ORIN_HOST:$DEPLOY_PATH/satellite_data/"
fi

# Sync helper scripts
echo ""
echo ">>> Syncing scripts..."
rsync -avz \
    "$PROJECT_DIR/scripts/" \
    "$ORIN_USER@$ORIN_HOST:$DEPLOY_PATH/scripts/"

# Pull latest images and restart
echo ""
echo ">>> Pulling latest Docker images and restarting services..."
ssh "$ORIN_USER@$ORIN_HOST" << EOF
    cd $DEPLOY_PATH
    
    # Pull latest images
    docker-compose pull || echo "Pull failed or images not on registry"
    
    # Stop existing containers
    docker-compose down || true
    
    # Start services
    docker-compose up -d
    
    # Show status
    echo ""
    echo "Container status:"
    docker-compose ps
EOF

echo ""
echo "=============================================="
echo "Deployment complete!"
echo "=============================================="
echo ""
echo "To view logs:"
echo "  ssh $ORIN_USER@$ORIN_HOST 'cd $DEPLOY_PATH && docker-compose logs -f'"
echo ""
echo "To stop services:"
echo "  ssh $ORIN_USER@$ORIN_HOST 'cd $DEPLOY_PATH && docker-compose down'"
