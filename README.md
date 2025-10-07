# ROS React Robot Control Panel

This project provides a web-based interface for controlling a ROS robot using React frontend and ROS backend.

## Architecture Overview

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

## How It Works

**Frontend (React)**: A web interface that connects to ROS via WebSocket, allowing users to control a robot with simple buttons (Forward, Backward, Left, Right, Stop).

**Backend (ROS)**: A robot simulation system that:

- Runs a virtual robot that responds to movement commands
- Publishes robot status updates
- Receives control commands from the web interface
- Uses ROS topics for communication between components

**Communication**: The React frontend and ROS backend communicate through a WebSocket bridge, translating web messages to ROS topics and vice versa.

## Project Structure

```
ros-react/
├── docker-compose.yml          # Main deployment configuration
├── robot-control-panel/        # React frontend
│   ├── Dockerfile
│   ├── package.json
│   ├── src/
│   │   ├── App.jsx
│   │   ├── App.css
│   │   └── index.js
│   └── public/
│       └── index.html
└── ros-backend-project/        # ROS backend
    ├── Dockerfile
    ├── ros_entrypoint.sh
    └── src/my_pkg/
        ├── launch/
        │   └── robot_sum.launch
        └── scripts/
            └── robot_sim_node.py
```

## Deployment

### Prerequisites

- Docker
- Docker Compose

### Quick Start

1. Clone the repository and navigate to the project directory:

   ```bash
   cd ros-react
   ```

2. Build and start both services:

   ```bash
   docker-compose up --build
   ```

3. Access the application:
   - **Frontend (React)**: http://localhost:3000
   - **ROS Bridge**: ws://localhost:9090

### Services

#### ROS Backend (`ros-backend`)

- **Port**: 9090 (ROS Bridge WebSocket)
- **Image**: Built from `ros-backend-project/Dockerfile`
- **Features**:
  - ROS Noetic with rosbridge server
  - Robot simulation node
  - WebSocket communication for web clients

#### React Frontend (`react-frontend`)

- **Port**: 3000 (Development server)
- **Image**: Built from `robot-control-panel/Dockerfile`
- **Features**:
  - React 18 with roslib integration
  - Real-time robot control interface
  - Connection status monitoring

### Development

#### Frontend Development

```bash
cd robot-control-panel
npm install
npm start
```

#### Backend Development

The ROS backend runs in a containerized environment. To modify the backend:

1. Edit files in `ros-backend-project/src/my_pkg/`
2. Rebuild the container: `docker-compose up --build ros-backend`

### Robot Control

The web interface provides:

- **Connection Status**: Shows ROS connection state
- **Robot Status**: Displays current robot state
- **Control Buttons**: Forward, Backward, Left, Right, Stop

### Troubleshooting

1. **Connection Issues**: Ensure both containers are running and the ROS backend is accessible
2. **Port Conflicts**: Check if ports 3000 and 9090 are available
3. **Build Issues**: Try `docker-compose down` then `docker-compose up --build`

### Stopping the Services

```bash
docker-compose down
```
