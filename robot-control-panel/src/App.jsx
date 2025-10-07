import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import './App.css';

function App() {
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');
  const [robotStatus, setRobotStatus] = useState('N/A');
  const ros = useRef(null);
  const statusListener = useRef(null);
  const cmdVelPublisher = useRef(null);

  useEffect(() => {
    ros.current = new ROSLIB.Ros({
      url: process.env.REACT_APP_ROS_WEBSOCKET_URL || 'ws://localhost:9090',
    });
    ros.current.on('connection', () => {
      setConnectionStatus('Connected');
      setupSubscribersAndPublishers();
    });
    ros.current.on('error', (error) => setConnectionStatus('Error'));
    ros.current.on('close', () => setConnectionStatus('Disconnected'));

    const setupSubscribersAndPublishers = () => {
      statusListener.current = new ROSLIB.Topic({
        ros: ros.current,
        name: '/robot_status',
        messageType: 'std_msgs/String',
      });
      statusListener.current.subscribe((message) =>
        setRobotStatus(message.data)
      );

      cmdVelPublisher.current = new ROSLIB.Topic({
        ros: ros.current,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });
    };
    return () => {
      if (statusListener.current) statusListener.current.unsubscribe();
      if (ros.current && ros.current.isConnected) ros.current.close();
    };
  }, []);

  const sendCommand = (linear, angular) => {
    if (!cmdVelPublisher.current) return;
    const twist = new ROSLIB.Message({
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular },
    });
    cmdVelPublisher.current.publish(twist);
  };

  return (
    <div className="App">
      <header className="App-header">
        <h1>Robot Remote Control</h1>
        <p>
          ROS Connection:{' '}
          <span className={connectionStatus.toLowerCase()}>
            {connectionStatus}
          </span>
        </p>
        <div className="status-display">
          <h2>
            Robot Status: <span>{robotStatus}</span>
          </h2>
        </div>
        <div className="controls">
          <button onClick={() => sendCommand(0.5, 0.0)}>Forward</button>
          <div>
            <button onClick={() => sendCommand(0.0, 0.5)}>Left</button>
            <button className="stop" onClick={() => sendCommand(0.0, 0.0)}>
              STOP
            </button>
            <button onClick={() => sendCommand(0.0, -0.5)}>Right</button>
          </div>
          <button onClick={() => sendCommand(-0.5, 0.0)}>Backward</button>
        </div>
      </header>
    </div>
  );
}
export default App;
