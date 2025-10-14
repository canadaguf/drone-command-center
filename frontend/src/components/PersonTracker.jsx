// frontend/src/components/PersonTracker.jsx
export default function PersonTracker() {
  // Later: this will come from WebSocket
  const mockPersons = [
    { id: 1, isFollowing: false },
    { id: 3, isFollowing: true },
  ];

  const toggleFollow = (id, currentlyFollowing) => {
    const action = currentlyFollowing ? 'stop_following' : 'follow';
    console.log(`Sending: ${action} for ID ${id}`);
    // Later: send via WebSocket
  };

  if (mockPersons.length === 0) {
    return <p>No persons detected.</p>;
  }

  return (
    <ul style={{ listStyle: 'none' }}>
      {mockPersons.map(person => (
        <li key={person.id} style={{ marginBottom: '12px', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <span>Person ID: <strong>{person.id}</strong></span>
          <button
            className={person.isFollowing ? 'btn-secondary' : 'btn-primary'}
            onClick={() => toggleFollow(person.id, person.isFollowing)}
          >
            {person.isFollowing ? 'Stop Following' : 'Follow'}
          </button>
        </li>
      ))}
    </ul>
  );
}