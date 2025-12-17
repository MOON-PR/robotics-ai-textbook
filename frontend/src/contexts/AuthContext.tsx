import React, { createContext, useContext, useState, ReactNode } from 'react';

interface AuthContextType {
  isAuthenticated: boolean;
  user: { username: string; level: string } | null;
  login: (username: string, level: string) => void;
  logout: () => void;
  updateUserLevel: (level: string) => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [isAuthenticated, setIsAuthenticated] = useState<boolean>(false);
  const [user, setUser] = useState<{ username: string; level: string } | null>(null);

  const login = (username: string, level: string) => {
    setIsAuthenticated(true);
    setUser({ username, level });
    // In a real app, you'd store tokens or user info in localStorage/sessionStorage
  };

  const logout = () => {
    setIsAuthenticated(false);
    setUser(null);
    // In a real app, you'd clear stored tokens/user info
  };

  const updateUserLevel = (level: string) => {
    if (user) {
      setUser({ ...user, level });
      // In a real app, you'd persist this change to a backend
    }
  };

  return (
    <AuthContext.Provider value={{ isAuthenticated, user, login, logout, updateUserLevel }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
