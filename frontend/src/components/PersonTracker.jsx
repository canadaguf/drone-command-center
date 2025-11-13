// frontend/src/components/PersonTracker.jsx
import { useState, useEffect } from 'react';

export default function PersonTracker({ detections, sendCommand, followedId, setFollowedId }) {
  const [personImages, setPersonImages] = useState({});
  const [commandStatus, setCommandStatus] = useState({}); // Track command status per person

  useEffect(() => {
    // Update images from detection data (thumbnails come as base64)
    const newImages = {};
    detections.forEach(person => {
      if (person.thumbnail) {
        // Convert base64 to data URL
        newImages[person.id] = `data:image/jpeg;base64,${person.thumbnail}`;
      } else {
        // Fallback to placeholder if no thumbnail available
        newImages[person.id] = `https://via.placeholder.com/80x120?text=CF+${person.id}`;
      }
    });
    setPersonImages(newImages);
  }, [detections]);

  const handleFollow = (id) => {
    setCommandStatus(prev => ({ ...prev, [id]: 'sending' }));
    
    if (followedId === id) {
      // Stop following
      sendCommand('stop_following', { target_id: id });
      setFollowedId(null);
      // Optimistically update status
      setTimeout(() => {
        setCommandStatus(prev => ({ ...prev, [id]: 'success' }));
        setTimeout(() => {
          setCommandStatus(prev => {
            const newStatus = { ...prev };
            delete newStatus[id];
            return newStatus;
          });
        }, 2000);
      }, 300);
    } else {
      // Start following (only if not already following someone)
      if (followedId !== null) {
        // Auto-stop previous target
        sendCommand('stop_following', { target_id: followedId });
      }
      sendCommand('follow', { target_id: id });
      setFollowedId(id);
      // Optimistically update status
      setTimeout(() => {
        setCommandStatus(prev => ({ ...prev, [id]: 'success' }));
        setTimeout(() => {
          setCommandStatus(prev => {
            const newStatus = { ...prev };
            delete newStatus[id];
            return newStatus;
          });
        }, 2000);
      }, 300);
    }
  };

  if (detections.length === 0) {
    return <p>No chairs detected.</p>;
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
                  alt={`Chair ${person.id}`}
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
              <div><strong>Chair ID:</strong> {person.id}</div>
              {person.class_name && (
                <div style={{ fontSize: '0.9em', color: '#666', marginTop: '4px' }}>
                  Class: {person.class_name}
                </div>
              )}
              {person.confidence && (
                <div style={{ fontSize: '0.9em', color: '#666', marginTop: '4px' }}>
                  Confidence: {(person.confidence * 100).toFixed(1)}%
                </div>
              )}
              <button
                className={isFollowed ? 'btn-secondary' : 'btn-primary'}
                onClick={() => handleFollow(person.id)}
                disabled={isDisabled || commandStatus[person.id] === 'sending'}
                style={{ marginTop: '8px', width: '100%' }}
              >
                {commandStatus[person.id] === 'sending' ? '⏳ Sending...' : 
                 commandStatus[person.id] === 'success' ? '✓ Done' :
                 isFollowed ? 'Stop Following' : 'Follow'}
              </button>
            </div>
          </div>
        );
      })}
    </div>
  );
}