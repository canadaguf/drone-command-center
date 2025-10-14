export default function PersonTracker({ detections, sendCommand }) {
  const toggleFollow = (id, currentlyFollowing) => {
    if (currentlyFollowing) {
      sendCommand('stop_following', { target_id: id });
    } else {
      sendCommand('follow', { target_id: id });
    }
  };

  // Use real detections
  if (detections.length === 0) {
    return <p>No persons detected.</p>;
  }

  return (
    <ul style={{ listStyle: 'none' }}>
      {detections.map(person => (
        <li key={person.id} style={{ marginBottom: '12px', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <span>Person ID: <strong>{person.id}</strong></span>
          <button
            className="control-btn"
            onClick={() => toggleFollow(person.id, person.is_following)}
          >
            {person.is_following ? 'Stop Following' : 'Follow'}
          </button>
        </li>
      ))}
    </ul>
  );
}