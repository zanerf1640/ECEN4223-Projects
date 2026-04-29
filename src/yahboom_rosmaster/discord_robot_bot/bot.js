// Discord Token: MTQ5ODUyMjc1MTE0MzUxNDIyMw.GLmDlc.Oe61MmCWY2TNbbxZd99lcJwvvdAjan5OYk3bxA

const { Client, GatewayIntentBits, Events } = require('discord.js');
const { exec } = require('child_process');
const WebSocket = require('ws');

const client = new Client({ 
    intents: [GatewayIntentBits.Guilds, GatewayIntentBits.GuildMessages, GatewayIntentBits.MessageContent] 
});

const DISCORD_TOKEN = 'MTQ5ODUyMjc1MTE0MzUxNDIyMw.GLmDlc.Oe61MmCWY2TNbbxZd99lcJwvvdAjan5OYk3bxA';

client.on(Events.MessageCreate, async (message) => {
    if (message.author.bot) return;

    const commandText = message.content.toLowerCase().trim();

    // 🛑 EMERGENCY STOP: Bypass AI with proper handshake [cite: 176]
    if (commandText === 'stop' || commandText === 'emergency stop') {
        const rosWs = new WebSocket('ws://127.0.0.1:9090');
        
        rosWs.on('open', () => {
            // Must advertise before publishing 
            rosWs.send(JSON.stringify({
                op: 'advertise',
                topic: '/openclaw/cmd_vel',
                type: 'geometry_msgs/msg/Twist'
            }));

            setTimeout(() => {
                rosWs.send(JSON.stringify({
                    op: 'publish',
                    topic: '/openclaw/cmd_vel',
                    msg: { linear: { x: 0.0, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.0 } }
                }));
                setTimeout(() => rosWs.close(), 100);
            }, 200);
        });
        
        message.reply('🛑 EMERGENCY STOP EXECUTED.');
        return; 
    }

    message.reply('🤖 Thinking...');
    exec(`openclaw agent --agent main --session-id main --message "${commandText}"`, { cwd: __dirname }, (error, stdout, stderr) => {
        if (error) return message.reply('⚠️ Error processing command.');
        if (stdout.trim().length > 0) {
            message.reply(`**AI:**\n\`\`\`text\n${stdout.substring(0, 1900)}\n\`\`\``);
        }
    });
});

client.login(DISCORD_TOKEN);