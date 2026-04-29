const WebSocket = require('ws');

console.log("Attempting to connect to rosbridge on port 9090...");
const ws = new WebSocket('ws://localhost:9090');

ws.on('open', () => {
    console.log("✅ Successfully connected to rosbridge!");
    
    // 1. ADVERTISE: Tell ROS 2 what we are about to do
    console.log("📢 Advertising topic /openclaw/cmd_vel...");
    const advertiseMsg = {
        op: 'advertise',
        topic: '/openclaw/cmd_vel',
        type: 'geometry_msgs/msg/Twist'
    };
    ws.send(JSON.stringify(advertiseMsg));
    
    // 2. WAIT: Give ROS 2 DDS half a second to shake hands with your Watchdog
    setTimeout(() => {
        console.log("🚀 Sending test movement command...");
        const publishMsg = {
            op: 'publish',
            topic: '/openclaw/cmd_vel', 
            msg: {
                linear: { x: 0.5, y: 0.0, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: 0.0 }
            }
        };
        ws.send(JSON.stringify(publishMsg));
    }, 500); // 500ms delay

    // 3. CLOSE after everything is done
    setTimeout(() => {
        console.log("🔌 Closing connection. Check your Watchdog terminal!");
        ws.close();
    }, 1500);
});

ws.on('error', (error) => {
    console.error(`❌ Connection failed: ${error.message}`);
});