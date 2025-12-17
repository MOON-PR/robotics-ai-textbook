import React from 'react';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';

const UserProfilePage: React.FC = () => {
  return (
    <Layout title="Profile" description="User Profile for AI-Native Textbook">
      <BrowserOnly fallback={<div>Loading...</div>}>
        {() => {
          // Client-only inner component to safely use hooks
          function Inner() {
            // eslint-disable-next-line @typescript-eslint/no-var-requires
            const { useAuth } = require('@site/src/contexts/AuthContext');
            const { Redirect } = require('@docusaurus/router');
            const { useState } = require('react');

            const { isAuthenticated, user, logout, updateUserLevel } = useAuth();

            if (!isAuthenticated || !user) {
              return <Redirect to="/login" />;
            }

            const handleLevelChange = (e: any) => {
              updateUserLevel(e.target.value);
            };

            return (
              <main style={{ padding: '20px' }}>
                <h1>User Profile</h1>
                <p><strong>Username:</strong> {user.username}</p>
                <div style={{ marginBottom: '15px' }}>
                  <label htmlFor="level" style={{ display: 'block', marginBottom: '5px' }}>Your Learning Level:</label>
                  <select
                    id="level"
                    value={user.level}
                    onChange={handleLevelChange}
                    style={{ padding: '8px', boxSizing: 'border-box' }}
                  >
                    <option value="Beginner">Beginner</option>
                    <option value="Intermediate">Intermediate</option>
                    <option value="Advanced">Advanced</option>
                  </select>
                </div>
                <button onClick={logout} style={{ padding: '10px', backgroundColor: '#dc3545', color: 'white', border: 'none', borderRadius: '5px', cursor: 'pointer' }}>
                  Logout
                </button>
              </main>
            );
          }

          return <Inner />;
        }}
      </BrowserOnly>
    </Layout>
  );
};

export default UserProfilePage;

export const prerender = false;
