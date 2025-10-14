// frontend/src/components/PersonTracker.jsx
import { useState, useEffect } from 'react';

export default function PersonTracker({ detections, sendCommand, followedId, setFollowedId }) {
  // Simulate image updates every 3s (replace with real data later)
  const [personImages, setPersonImages] = useState({});

  useEffect(() => {
    // Mock: generate/update preview every 3s
    const interval = setInterval(() => {
      const newImages = {};
      detections.forEach(person => {
        // In real app: this would come from drone (e.g., base64 snapshot)
        // For now: use placeholder with ID
        newImages[person.id] = `https://via.placeholder.com/80x120?text=ID+${person.id}`;
      });
      setPersonImages(newImages);
    }, 3000);

    // Initial load
    const initial = {};
    detections.forEach(p => {
      initial[p.id] = `https://via.placeholder.com/80x120?text=ID+${p.id}`;
    });
    setPersonImages(initial);

    return () => clearInterval(interval);
  }, [detections]);

  const handleFollow = (id) => {
    if (followedId === id) {
      // Stop following
      sendCommand('stop_following', { target_id: id });
      setFollowedId(null);
    } else {
      // Start following (only if not already following someone)
      if (followedId !== null) {
        // Optional: auto-stop previous? Or enforce user to stop first?
        // We'll allow direct switch for better UX
        sendCommand('stop_following', { target_id: followedId });
      }
      sendCommand('follow', { target_id: id });
      setFollowedId(id);
    }
  };

  if (detections.length === 0) {
    return <p>No persons detected.</p>;
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '16px' }}>
      {detections.map(person => {
        const isFollowed = followedId === person.id;
        const isDisabled = followedId !== null && !isFollowed;

        return (
          <div
            key={person.id}
            style={{
              border: isFollowed ? '2px solid #3498db' : '1px solid #eee',
              borderRadius: '8px',
              padding: '12px',
              display: 'flex',
              alignItems: 'center',
              gap: '16px',
              backgroundColor: isFollowed ? '#f0f8ff' : 'white',
            }}
          >
            {/* Preview Image */}
            <div style={{ width: '80px', height: '120px', overflow: 'hidden', borderRadius: '4px' }}>
              {personImages[person.id] ? (
                <img
                  src={personImages[person.id]}
                  alt={`Person ${person.id}`}
                  style={{ width: '100%', height: '100%', objectFit: 'cover' }}
                />
              ) : (
                <div style={{ width: '100%', height: '100%', background: '#f5f5f5', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                  ⏳
                </div>
              )}
            </div>

            {/* Info & Button */}
            <div style={{ flex: 1 }}>
              <div><strong>Person ID:</strong> {person.id}</div>
              <button
                className={isFollowed ? 'btn-secondary' : 'btn-primary'}
                onClick={() => handleFollow(person.id)}
                disabled={isDisabled}
                style={{ marginTop: '8px', width: '100%' }}
              >
                {isFollowed ? 'Stop Following' : 'Follow'}
              </button>
            </div>
          </div>
        );
      })}
    </div>
  );
}