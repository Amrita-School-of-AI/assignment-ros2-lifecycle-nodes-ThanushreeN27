#!/bin/bash

# 1. Source the ROS 2 environment
source /opt/ros/jazzy/setup.bash

# 2. Source the local workspace (if it was built)
if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  echo "❌ Error: install/setup.bash not found. Did the build fail?"
  exit 1
fi

# 3. Start the lifecycle node in background
echo "Starting lifecycle node..."
ros2 run ros2_lifecycle lifecycle_sensor > output.log 2>&1 &
NODE_PID=$!
sleep 2

# 4. Check initial state
echo "Checking initial state..."
STATE=$(ros2 lifecycle get /lifecycle_sensor 2>/dev/null | grep -o 'unconfigured\|inactive\|active')
if [ "$STATE" != "unconfigured" ]; then
  echo "❌ Initial state should be 'unconfigured', got '$STATE'"
  kill $NODE_PID 2>/dev/null || true
  exit 1
fi
echo "✅ Initial state is unconfigured"

# 5. Configure the node
echo "Configuring node..."
ros2 lifecycle set /lifecycle_sensor configure
sleep 1

STATE=$(ros2 lifecycle get /lifecycle_sensor 2>/dev/null | grep -o 'unconfigured\|inactive\|active')
if [ "$STATE" != "inactive" ]; then
  echo "❌ State after configure should be 'inactive', got '$STATE'"
  kill $NODE_PID 2>/dev/null || true
  exit 1
fi
echo "✅ State after configure is inactive"

# 6. Activate the node
echo "Activating node..."
ros2 lifecycle set /lifecycle_sensor activate
sleep 2

STATE=$(ros2 lifecycle get /lifecycle_sensor 2>/dev/null | grep -o 'unconfigured\|inactive\|active')
if [ "$STATE" != "active" ]; then
  echo "❌ State after activate should be 'active', got '$STATE'"
  kill $NODE_PID 2>/dev/null || true
  exit 1
fi
echo "✅ State after activate is active"

# 7. Check if publishing
echo "Checking if publishing..."
ros2 topic echo /sensor_data --once > topic_output.log 2>&1 &
ECHO_PID=$!
sleep 3
kill $ECHO_PID 2>/dev/null || true

if grep -q "data:" topic_output.log; then
  echo "✅ Node is publishing sensor data when active"
else
  echo "❌ Node is not publishing sensor data"
  kill $NODE_PID 2>/dev/null || true
  exit 1
fi

# 8. Deactivate the node
echo "Deactivating node..."
ros2 lifecycle set /lifecycle_sensor deactivate
sleep 1

STATE=$(ros2 lifecycle get /lifecycle_sensor 2>/dev/null | grep -o 'unconfigured\|inactive\|active')
if [ "$STATE" != "inactive" ]; then
  echo "❌ State after deactivate should be 'inactive', got '$STATE'"
  kill $NODE_PID 2>/dev/null || true
  exit 1
fi
echo "✅ State after deactivate is inactive"

# Cleanup
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true

echo ""
echo "Node output:"
cat output.log

# Check for lifecycle messages
if grep -q "configured" output.log && grep -q "activated" output.log && grep -q "deactivated" output.log; then
  echo "✅ All lifecycle callbacks executed correctly"
  exit 0
else
  echo "❌ Not all lifecycle callbacks were executed"
  exit 1
fi
