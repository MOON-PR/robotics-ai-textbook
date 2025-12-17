import React, { useState } from 'react';
import styles from './Chatbot.module.css';

const Chatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<{ text: string; sender: 'user' | 'bot' }[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async () => {
    if (input.trim()) {
      const userMessage = { text: input, sender: 'user' as const };
      setMessages((prevMessages) => [...prevMessages, userMessage]);
      setInput('');
      setIsLoading(true);

      try {
        const BACKEND_URL = (typeof window !== 'undefined' && (window as any).__BACKEND_URL__) ? (window as any).__BACKEND_URL__ : 'http://localhost:8000';
        console.log("BACKEND_URL", BACKEND_URL);
        const response = await fetch(`${BACKEND_URL.replace(/\/+$/, '')}/chat`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ query: userMessage.text }),
        });
        console.log("FETCH RESPONSE", response);
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json();
        console.log("API RESPONSE", data);
        const botResponse = { text: data.response, sender: 'bot' as const };
        setMessages((prevMessages) => [...prevMessages, botResponse]);

      } catch (error) {
        console.error('Error sending message to backend:', error);
        const errorMessage = { text: 'Error: Could not connect to the chatbot. Please ensure the backend is running.', sender: 'bot' as const };
        setMessages((prevMessages) => [...prevMessages, errorMessage]);
      } finally {
        setIsLoading(false);
      }
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      <button className={styles.toggleButton} onClick={toggleChat}>
        {isOpen ? 'Close Chat' : 'Open Chat'}
      </button>

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.messages}>
            {messages.map((msg, index) => (
              <div key={index} className={`${styles.message} ${styles[msg.sender]}`}>
                {msg.text}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.bot}`}>
                🤔 Thinking...
              </div>
            )}
          </div>
          <div className={styles.inputArea}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === 'Enter') {
                  handleSendMessage();
                }
              }}
              placeholder="Type your message..."
            />
            <button onClick={handleSendMessage}>Send</button>
          </div>
        </div>
      )}
    </div>
  );
};

export default Chatbot;
