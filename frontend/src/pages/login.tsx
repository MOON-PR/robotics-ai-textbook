import React from 'react';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';

const LoginPage: React.FC = () => {
  return (
    <Layout title="Login" description="Login to AI-Native Textbook">
      <BrowserOnly fallback={<div>Loading...</div>}>
        {() => {
          const { useState } = React as any;
          // Inner component to use hooks safely on the client
          function Inner() {
            // Import hooks and context inside client-only component
            // eslint-disable-next-line @typescript-eslint/no-var-requires
            const { useState: useStateHook } = require('react');
            const { useAuth } = require('@site/src/contexts/AuthContext');
            const { Redirect } = require('@docusaurus/router');

            const { isAuthenticated, login } = useAuth();
            const [username, setUsername] = useStateHook('');
            const [level, setLevel] = useStateHook('Beginner');

            if (isAuthenticated) {
              return <Redirect to="/" />;
            }

            const handleLogin = (e: any) => {
              e.preventDefault();
              if (username) {
                login(username, level);
              }
            };

            return (
              <main style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '80vh' }}>
                <div style={{ padding: '20px', border: '1px solid #ccc', borderRadius: '8px', maxWidth: '400px', width: '100%' }}>
                  <h1>Login</h1>
                  <form onSubmit={handleLogin}>
                    <div style={{ marginBottom: '15px' }}>
                      <label htmlFor="username" style={{ display: 'block', marginBottom: '5px' }}>Username:</label>
                      <input
                        type="text"
                        id="username"
                        value={username}
                        onChange={(e: any) => setUsername(e.target.value)}
                        style={{ width: '100%', padding: '8px', boxSizing: 'border-box' }}
                        required
                      />
                    </div>
                    <div style={{ marginBottom: '15px' }}>
                      <label htmlFor="level" style={{ display: 'block', marginBottom: '5px' }}>Learning Level:</label>
                      <select
                        id="level"
                        value={level}
                        onChange={(e: any) => setLevel(e.target.value)}
                        style={{ width: '100%', padding: '8px', boxSizing: 'border-box' }}
                      >
                        <option value="Beginner">Beginner</option>
                        <option value="Intermediate">Intermediate</option>
                        <option value="Advanced">Advanced</option>
                      </select>
                    </div>
                    <button type="submit" style={{ width: '100%', padding: '10px', backgroundColor: '#007bff', color: 'white', border: 'none', borderRadius: '5px', cursor: 'pointer' }}>
                      Login
                    </button>
                  </form>
                </div>
              </main>
            );
          }

          return <Inner />;
        }}
      </BrowserOnly>
    </Layout>
  );
};

export default LoginPage;

export const prerender = false;
