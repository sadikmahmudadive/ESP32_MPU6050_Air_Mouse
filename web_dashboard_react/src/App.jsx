import React, { useEffect, useState } from 'react';
import { db } from './firebase';
import { ref, query, limitToLast, onValue, remove, orderByChild, endAt, get } from 'firebase/database';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import { Activity, Battery, Wifi, MousePointer2 } from 'lucide-react';

function App() {
  const [logs, setLogs] = useState([]);
  const [history, setHistory] = useState([]);
  const [mlEvents, setMlEvents] = useState([]);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    // Listen for Logs (Sensor Data)
    const logsRef = query(ref(db, 'logs'), limitToLast(50));
    const unsubscribeLogs = onValue(logsRef, (snapshot) => {
      const data = snapshot.val();
      if (data) {
        const list = Object.keys(data).map(key => ({ ...data[key], id: key }));
        // Sort by timestamp if needed, but firebase returns sorted by key usually
        setLogs(list);
        setConnected(true);
      }
    });

    // Listen for Connection History
    const historyRef = query(ref(db, 'history'), limitToLast(10));
    const unsubscribeHistory = onValue(historyRef, (snapshot) => {
      const data = snapshot.val();
      if (data) {
        const list = Object.keys(data).map(key => ({ ...data[key], id: key })).reverse();
        setHistory(list);
      }
    });

    // Listen for ML Events
    const mlRef = query(ref(db, 'ml_events'), limitToLast(10));
    const unsubscribeMl = onValue(mlRef, (snapshot) => {
      const data = snapshot.val();
      if (data) {
        const list = Object.keys(data).map(key => ({ ...data[key], id: key })).reverse();
        setMlEvents(list);
      }
    });

    return () => {
      unsubscribeLogs();
      unsubscribeHistory();
      unsubscribeMl();
    };
  }, []);

  // Cleanup old data (older than 1 hour)
  useEffect(() => {
    const cleanupOldData = async () => {
      const oneHourAgo = Date.now() - 3600000;
      // Query logs older than 1 hour
      const oldLogsRef = query(ref(db, 'logs'), orderByChild('timestamp'), endAt(oneHourAgo));
      
      try {
        const snapshot = await get(oldLogsRef);
        if (snapshot.exists()) {
          let count = 0;
          snapshot.forEach((child) => {
            remove(child.ref);
            count++;
          });
          console.log(`Cleaned up ${count} old log entries`);
        }
      } catch (error) {
        console.error("Cleanup error:", error);
      }
    };

    // Run on mount
    cleanupOldData();
    // And every 10 minutes
    const interval = setInterval(cleanupOldData, 600000);
    return () => clearInterval(interval);
  }, []);

  const latest = logs.length > 0 ? logs[logs.length - 1] : null;

  return (
    <div className="min-h-screen bg-gray-100 p-8">
      <div className="max-w-7xl mx-auto space-y-6">
        
        {/* Header */}
        <div className="flex justify-between items-center bg-white p-6 rounded-xl shadow-sm">
          <div>
            <h1 className="text-2xl font-bold text-gray-800 flex items-center gap-2">
              <MousePointer2 className="w-8 h-8 text-blue-600" />
              ESP32 Air Mouse Dashboard
            </h1>
            <p className="text-gray-500 text-sm mt-1">Real-time telemetry & ML insights</p>
          </div>
          <div className="flex items-center gap-4">
            <div className={`flex items-center gap-2 px-4 py-2 rounded-full ${connected ? 'bg-green-100 text-green-700' : 'bg-red-100 text-red-700'}`}>
              <Wifi className="w-4 h-4" />
              <span className="font-medium">{connected ? 'Live' : 'Connecting...'}</span>
            </div>
          </div>
        </div>

        {/* Key Metrics Cards */}
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          <div className="bg-white p-6 rounded-xl shadow-sm border-l-4 border-blue-500">
            <div className="flex justify-between items-start">
              <div>
                <p className="text-gray-500 text-sm font-medium">Orientation</p>
                <h3 className="text-2xl font-bold text-gray-800 mt-1">
                  P: {latest?.pitch?.toFixed(1)}°
                </h3>
                <p className="text-gray-600">R: {latest?.roll?.toFixed(1)}°</p>
              </div>
              <Activity className="w-6 h-6 text-blue-500" />
            </div>
          </div>

          <div className="bg-white p-6 rounded-xl shadow-sm border-l-4 border-purple-500">
            <div className="flex justify-between items-start">
              <div>
                <p className="text-gray-500 text-sm font-medium">Battery</p>
                <h3 className="text-2xl font-bold text-gray-800 mt-1">
                  {latest?.battery_v?.toFixed(2)} V
                </h3>
                <p className="text-gray-600">{(latest?.battery_v > 3.4 ? 'Good' : 'Low')}</p>
              </div>
              <Battery className="w-6 h-6 text-purple-500" />
            </div>
          </div>

          <div className="bg-white p-6 rounded-xl shadow-sm border-l-4 border-orange-500">
            <div className="flex justify-between items-start">
              <div>
                <p className="text-gray-500 text-sm font-medium">Last Event</p>
                <h3 className="text-lg font-bold text-gray-800 mt-1">
                  {mlEvents[0]?.event || 'None'}
                </h3>
                <p className="text-xs text-gray-500 mt-1">
                  {mlEvents[0] ? new Date(mlEvents[0].timestamp).toLocaleTimeString() : '--:--'}
                </p>
              </div>
              <Activity className="w-6 h-6 text-orange-500" />
            </div>
          </div>
        </div>

        {/* Charts */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          <div className="bg-white p-6 rounded-xl shadow-sm">
            <h3 className="text-lg font-semibold text-gray-800 mb-4">Pitch & Roll History</h3>
            <div className="h-64">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={logs}>
                  <CartesianGrid strokeDasharray="3 3" stroke="#f0f0f0" />
                  <XAxis dataKey="timestamp" tick={false} />
                  <YAxis domain={['auto', 'auto']} />
                  <Tooltip labelFormatter={(t) => new Date(t).toLocaleTimeString()} />
                  <Legend />
                  <Line type="monotone" dataKey="pitch" stroke="#3b82f6" strokeWidth={2} dot={false} />
                  <Line type="monotone" dataKey="roll" stroke="#8b5cf6" strokeWidth={2} dot={false} />
                </LineChart>
              </ResponsiveContainer>
            </div>
          </div>

          <div className="bg-white p-6 rounded-xl shadow-sm">
            <h3 className="text-lg font-semibold text-gray-800 mb-4">Gyroscope Activity (Raw)</h3>
            <div className="h-64">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={logs}>
                  <CartesianGrid strokeDasharray="3 3" stroke="#f0f0f0" />
                  <XAxis dataKey="timestamp" tick={false} />
                  <YAxis />
                  <Tooltip labelFormatter={(t) => new Date(t).toLocaleTimeString()} />
                  <Legend />
                  <Line type="monotone" dataKey="gx" stroke="#ef4444" dot={false} />
                  <Line type="monotone" dataKey="gy" stroke="#10b981" dot={false} />
                  <Line type="monotone" dataKey="gz" stroke="#f59e0b" dot={false} />
                </LineChart>
              </ResponsiveContainer>
            </div>
          </div>
        </div>

        {/* Tables Grid */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          {/* Connection History */}
          <div className="bg-white rounded-xl shadow-sm overflow-hidden">
            <div className="px-6 py-4 border-b border-gray-100">
              <h3 className="font-semibold text-gray-800">Connection History</h3>
            </div>
            <div className="overflow-x-auto">
              <table className="w-full text-sm text-left">
                <thead className="bg-gray-50 text-gray-600">
                  <tr>
                    <th className="px-6 py-3">Time</th>
                    <th className="px-6 py-3">Event</th>
                    <th className="px-6 py-3">IP Address</th>
                  </tr>
                </thead>
                <tbody className="divide-y divide-gray-100">
                  {history.map((item) => (
                    <tr key={item.id} className="hover:bg-gray-50">
                      <td className="px-6 py-3">{new Date(item.timestamp).toLocaleString()}</td>
                      <td className="px-6 py-3">
                        <span className="px-2 py-1 bg-green-100 text-green-700 rounded-full text-xs font-medium">
                          {item.event}
                        </span>
                      </td>
                      <td className="px-6 py-3 font-mono text-gray-600">{item.ip}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>

          {/* ML Events */}
          <div className="bg-white rounded-xl shadow-sm overflow-hidden">
            <div className="px-6 py-4 border-b border-gray-100">
              <h3 className="font-semibold text-gray-800">ML Gesture Events</h3>
            </div>
            <div className="overflow-x-auto">
              <table className="w-full text-sm text-left">
                <thead className="bg-gray-50 text-gray-600">
                  <tr>
                    <th className="px-6 py-3">Time</th>
                    <th className="px-6 py-3">Gesture</th>
                    <th className="px-6 py-3">Intensity (G-Mag)</th>
                  </tr>
                </thead>
                <tbody className="divide-y divide-gray-100">
                  {mlEvents.map((item) => (
                    <tr key={item.id} className="hover:bg-gray-50">
                      <td className="px-6 py-3">{new Date(item.timestamp).toLocaleString()}</td>
                      <td className="px-6 py-3">
                        <span className="px-2 py-1 bg-purple-100 text-purple-700 rounded-full text-xs font-medium">
                          {item.event}
                        </span>
                      </td>
                      <td className="px-6 py-3">{item.gmag?.toFixed(0)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>
        </div>

      </div>
    </div>
  );
}

export default App;
