# ROS React Robot Control Panel

A web-based interface for controlling a ROS robot using React frontend and ROS backend. This project demonstrates real-time robot control through a modern web interface with WebSocket communication.

## Project Synopsis

This application allows you to control a virtual robot through your web browser. You can:

- **Connect** to a ROS robot simulation via WebSocket
- **Control** robot movement with simple buttons (Forward, Backward, Left, Right, Stop)
- **Monitor** real-time robot status and connection state
- **Experience** seamless communication between web frontend and ROS backend

## Screenshots

### Disconnected State

![Disconnected State](Screenshot%202025-10-07%20at%2020.51.54.png)
_The interface shows "Disconnected" status when not connected to ROS backend_

### Connected & Functional State

![Connected State](Screenshot%202025-10-07%20at%2020.54.40.png)
_The interface shows "Connected" status and displays real-time robot status updates_

## Architecture Overall

```
┌─────────────────┐    WebSocket     ┌─────────────────┐    ROS Topics    ┌─────────────────┐
│                 │    (Port 9090)   │                 │                  │                 │
│  React Frontend │◄─────────────────►│  ROS Bridge     │◄─────────────────►│  Robot Simulator│
│  (Port 3000)    │                  │  Server         │                  │  Node           │
│                 │                  │                 │                  │                 │
└─────────────────┘                  └─────────────────┘                  └─────────────────┘
        │                                     │                                     │
        │                                     │                                     │
        ▼                                     ▼                                     ▼
   User Interface                    WebSocket Gateway                    Robot Logic
   - Control Buttons                 - /robot_status topic               - Status Publisher
   - Connection Status               - /cmd_vel topic                    - Command Subscriber
   - Real-time Updates               - Message Translation               - Movement Simulation
```

## Fullstack functionalities

### Frontend (React)

- **Web Interface**: Responsive UI with control buttons
- **Real-time Connection**: WebSocket connection to ROS backend
- **Status Monitoring**: Live updates of robot state and connection status
- **Command Sending**: Publishes movement commands to robot

### Backend (ROS)

- **Robot Simulation**: Virtual robot that responds to movement commands
- **Status Publishing**: Continuously publishes robot state updates
- **Command Processing**: Receives and processes movement commands
- **WebSocket Bridge**: Translates between web messages and ROS topics

### Communication Flow

```
User clicks "Forward" → React → WebSocket → ROS Bridge → Robot Simulator → Status Update → Back to UI
```

## 📁 Project Structure

```
ros-react/
├── docker-compose.yml              # Main deployment configuration
├── README.md                       # This file
├── Screenshot 2025-10-07 at 20.51.54.png  # Disconnected state
├── Screenshot 2025-10-07 at 20.54.40.png  # Connected state
├── robot-control-panel/            # React frontend
│   ├── Dockerfile
│   ├── package.json
│   ├── src/
│   │   ├── App.jsx                 # Main React component
│   │   ├── App.css                 # Styling
│   │   └── index.js                # Entry point
│   └── public/
│       └── index.html
└── ros-backend-project/            # ROS backend
    ├── Dockerfile
    ├── ros_entrypoint.sh           # ROS environment setup
    ├── start_ros.sh                # Startup script
    └── src/my_pkg/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── robot_sum.launch    # ROS launch configuration
        └── scripts/
            └── robot_sim_node.py   # Robot simulation logic
```

## Quick Start

### Prerequisites

- Docker
- Docker Compose

### Installation & Running

1. **Clone and navigate to the project**:

   ```bash
   cd ros-react
   ```

2. **Start the application**:

   ```bash
   docker-compose up --build
   ```

3. **Access the application**:

   - **Frontend**: http://localhost:3000
   - **ROS Bridge**: ws://localhost:9090

4. **Control the robot**:
   - Wait for "Connected" status
   - Use the control buttons to move the robot
   - Watch the robot status update in real-time

## 🛠️ Development

### Frontend Development

```bash
cd robot-control-panel
npm install
npm start
```

### Backend Development

The ROS backend runs in a containerized environment. To modify:

1. Edit files in `ros-backend-project/src/my_pkg/`
2. Rebuild: `docker-compose up --build ros-backend`

### Key ROS Concepts

- **Topics**: Communication channels (`/robot_status`, `/cmd_vel`)
- **Messages**: Data structures (`std_msgs/String`, `geometry_msgs/Twist`)
- **Nodes**: Individual programs (robot simulator, rosbridge server)
- **Launch Files**: Configuration for starting multiple nodes

## 🎮 Robot Control Interface

The web interface provides:

- **Connection Status**: Real-time ROS connection state
- **Robot Status**: Current robot state (Idle, Moving Forward, etc.)
- **Control Buttons**:
  - Forward/Backward (linear movement)
  - Left/Right (angular movement)
  - Stop (emergency stop)

## Troubleshooting

### Common Issues that I faced.

1. **Connection Issues**:

   - Ensure both containers are running
   - Check if ROS backend is accessible on port 9090
   - Verify WebSocket connection in browser dev tools

2. **Port Conflicts**:

   - Check if ports 3000 and 9090 are available
   - Use `lsof -i :3000` and `lsof -i :9090` to check
   - Fix the version of the docker build compose

3. **Build Issues**:

   ```bash
   docker-compose down
   docker-compose up --build
   ```

4. **Container Issues**:
   ```bash
   docker-compose logs ros-backend
   docker-compose logs react-frontend
   ```

## Stopping the Services

```bash
docker-compose down
```

## 🔄 Pausing/Resuming Services

### Pause ROS Backend

```bash
docker-compose pause ros-backend
```

### Resume ROS Backend

```bash
docker-compose unpause ros-backend
```

### Pause All Services

```bash
docker-compose pause
```


---

**Happy Robot Controlling! 🤖**
