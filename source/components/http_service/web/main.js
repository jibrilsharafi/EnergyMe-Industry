document.addEventListener('DOMContentLoaded', function() {
    // DOM elements
    const statusEl = document.getElementById('status');
    const voltageL1El = document.getElementById('voltage-l1');
    const currentL1El = document.getElementById('current-l1');
    const powerL1El = document.getElementById('power-l1');
    const totalEnergyEl = document.getElementById('total-energy');
    const activePowerEl = document.getElementById('active-power');
    const powerFactorEl = document.getElementById('power-factor');
    const muxChannelEl = document.getElementById('mux-channel');
    const channelSelectEl = document.getElementById('channel-select');
    const setChannelBtn = document.getElementById('set-channel');
    const refreshDataBtn = document.getElementById('refresh-data');
    const restartDeviceBtn = document.getElementById('restart-device');
    
    // Function to fetch device data
    async function fetchData() {
        try {
            const response = await fetch('/api/readings');
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            
            const data = await response.json();
            updateUI(data);
            
            // Set status to online
            statusEl.className = 'online';
            statusEl.textContent = 'System Status: Online';
        } catch (error) {
            console.error('Error fetching data:', error);
            
            // Set status to offline if fetch fails
            statusEl.className = 'offline';
            statusEl.textContent = 'System Status: Offline';
        }
    }
    
    // Function to update UI with new data
    function updateUI(data) {
        if (data.voltage) voltageL1El.textContent = data.voltage.l1.toFixed(1);
        if (data.current) currentL1El.textContent = data.current.l1.toFixed(2);
        if (data.power) powerL1El.textContent = data.power.l1.toFixed(0);
        
        if (data.energy) totalEnergyEl.textContent = data.energy.total.toFixed(2);
        if (data.activePower) activePowerEl.textContent = data.activePower.total.toFixed(0);
        if (data.powerFactor) powerFactorEl.textContent = data.powerFactor.total.toFixed(2);
        
        if (data.mux !== undefined) muxChannelEl.textContent = data.mux.channel;
    }
    
    // Set channel button click handler
    setChannelBtn.addEventListener('click', async function() {
        const channel = channelSelectEl.value;
        if (!channel) return;
        
        try {
            const response = await fetch('/api/mux', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ channel: parseInt(channel) })
            });
            
            if (!response.ok) {
                throw new Error('Failed to set channel');
            }
            
            // Update display after successful change
            muxChannelEl.textContent = channel;
        } catch (error) {
            console.error('Error setting channel:', error);
            alert('Failed to set channel. Please try again.');
        }
    });
    
    // Refresh data button click handler
    refreshDataBtn.addEventListener('click', function() {
        fetchData();
    });
    
    // Restart device button click handler
    restartDeviceBtn.addEventListener('click', async function() {
        if (!confirm('Are you sure you want to restart the device?')) {
            return;
        }
        
        try {
            const response = await fetch('/api/restart', {
                method: 'POST'
            });
            
            if (!response.ok) {
                throw new Error('Failed to restart device');
            }
            
            statusEl.className = 'offline';
            statusEl.textContent = 'System Status: Restarting...';
            
            // Wait for device to restart before attempting to reconnect
            setTimeout(fetchData, 30000);
        } catch (error) {
            console.error('Error restarting device:', error);
            alert('Failed to restart device. Please try again.');
        }
    });
    
    // Fetch data initially and set up periodic refresh
    fetchData();
    setInterval(fetchData, 10000); // Refresh every 10 seconds
});
