/*

instructions:
First you are going to create a folder inside your computer.
Open the folder using VSCode.
Open a terminal CMD

Type the following commands, one by one.

npm init -y
npm install express mysql2
npm install mysql2
npm install mqtt

Open MAMP or similar.
Open phpmyadmin or similar

Create a database called data_example

Create a table with four rows called values. Configure each row with the following setup:

----------
ID INT AUTO_INCREMENT PRIMARY KEY,
numeric_value INT NOT NULL,
source_id INT NOT NULL,
timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
-------------

Start Mosquitto in localhost.


Test this server by running the command: "node server.js" on the terminal (no "").

You should see the messages  "Server running on http://localhost:3001"
Also the message: "Press CTRL+C to stop the server."

Press CTRL+C to stop the server.

To test the server:

POST: to create a register
curl -X POST http://localhost:3001/api/values -H "Content-Type: application/json" -d '{"numeric_value": 42, "source_id": 101}'

GET: to print a register
curl -X GET http://localhost:3001/api/values/1

PUT: to update a register
curl -X PUT http://localhost:3001/api/values/1 -H "Content-Type: application/json" -d "{\"numeric_value\": 99, \"source_id\": 202}"

DELETE:
curl -X DELETE http://localhost:3001/api/values/1


MQTT Part

Open MQTTX
Connect to localhost mosquitoo

Send a message to:
Topic: sensors/data --> now called: scara/telemetry
Message: (this json)

{
  "numeric_value": 85,
  "source_id": 404
}

The data should be added to the database

Test with different data.


*/


const mqtt = require('mqtt');

const express = require('express');
const mysql = require('mysql2/promise');

const app = express();
app.use(express.json()); 

const PORT = 3001;

// Database connection
const dbConfig = {
    host: process.env.DB_HOST || '127.0.0.1',
    user: process.env.DB_USER || 'scara',
    password: process.env.DB_PASS,
    database: process.env.DB_NAME || 'scara'
};
if (!dbConfig.password || !process.env.MQTT_PASS) {
    console.error('Set DB_PASS and MQTT_PASS in .env');
    process.exit(1);
}

// Connect to the database
const pool = mysql.createPool(dbConfig);


//REST api commands. post --> new, PUt--> update, Get-->read , delete-->delete

app.post('/api/values', async (req, res) => {
    const { numeric_value, source_id } = req.body;
    
    if (numeric_value === undefined || source_id === undefined) {
        return res.status(400).json({ error: 'Both numeric_value and source_id are required!' });
    }

    try {
        const query = 'INSERT INTO `values` (numeric_value, source_id) VALUES (?, ?)';
        const [result] = await pool.execute(query, [numeric_value, source_id]);
        
        res.status(201).json({ 
            message: 'Record created successfully', 
            id: result.insertId 
        });

        console.log('Record inserted. s=' + source_id + '  data=' + numeric_value);
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});


app.get('/api/values/:id', async (req, res) => {
    const { id } = req.params;

    try {
        const query = 'SELECT * FROM `values` WHERE ID = ?';
        const [rows] = await pool.execute(query, [id]);

        if (rows.length === 0) {
            return res.status(404).json({ error: 'Record not found' });
        }
        
        const record = rows[0];
        console.log('Record read. ID: ${id} Value: ${record.numeric_value}');
       
        res.json(rows); 
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});


app.put('/api/values/:id', async (req, res) => {
    const { id } = req.params;
    const { numeric_value, source_id } = req.body;

    if (numeric_value === undefined || source_id === undefined) {
        return res.status(400).json({ error: 'Both numeric_value and source_id are required for update!' });
    }

    try {
        const query = 'UPDATE `values` SET numeric_value = ?, source_id = ? WHERE ID = ?';
        const [result] = await pool.execute(query, [numeric_value, source_id, id]);
        
        if (result.affectedRows === 0) {
            return res.status(404).json({ error: 'Record not found to update' });
        }
        res.json({ message: 'Record updated successfully' });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

app.delete('/api/values/:id', async (req, res) => {
    const { id } = req.params;

    try {
        const query = 'DELETE FROM `values` WHERE ID = ?';
        const [result] = await pool.execute(query, [id]);

        if (result.affectedRows === 0) {
            return res.status(404).json({ error: 'Record not found to delete' });
        }

        res.json({ message: 'Record deleted successfully' });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

//MQTT part

const mqttClient = mqtt.connect(process.env.MQTT_URL || 'mqtt://localhost:1883', {
    username: process.env.MQTT_USER || 'scara',
    password: process.env.MQTT_PASS
});
mqttClient.on('error', (e) => console.error('MQTT error:', e.message));

const MQTT_TOPIC = 'scara/telemetry';

mqttClient.on('connect', () => {
    console.log(`Connected to Mosquitto broker. Subscribing to topic: "${MQTT_TOPIC}"`);
    mqttClient.subscribe(MQTT_TOPIC);
});

mqttClient.on('message', async (topic, message) => {
    try {
        const payload = JSON.parse(message.toString());
        
        // Extract the variables from the incoming JSON object
        const { numeric_value, source_id } = payload;

        // Basic validation check for students
        if (numeric_value === undefined || source_id === undefined) {
            console.log('MQTT Error: Incoming JSON is missing numeric_value or source_id');
            return;
        }

        const query = 'INSERT INTO `values` (numeric_value, source_id) VALUES (?, ?)';
        const [result] = await pool.execute(query, [numeric_value, source_id]);

        console.log(`MQTT Record Inserted. ID: ${result.insertId} | Data: ${numeric_value} | Source: ${source_id}`);

    } catch (error) {
        console.error('MQTT Processing Error:', error.message);
    }
});



// Start server
const server = app.listen(PORT, () => {
    console.log(`Server running on http://localhost:${PORT}`);
    console.log('Press CTRL+C to stop the server.');
});

// --- (CTRL+C Handler) --- SIGINT is generated when you press CTRL+C
process.on('SIGINT', async () => {
    console.log('\n[CTRL+C detected]');

    server.close(() => {
        console.log('Server stopped.');
    });

    try {
        await pool.end();
        console.log('Database closed.');
    } catch (err) {
        console.error('Error closing database connections:', err.message);
    }

    console.log('Goodbye!');
    process.exit(0); // Exit the terminal process successfully
});
