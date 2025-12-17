import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import Chatbot from './index';

// Mock the fetch API
global.fetch = jest.fn((url, options) => {
  if (url === 'http://localhost:8000/chat') {
    return Promise.resolve({
      ok: true,
      json: () => Promise.resolve({ response: 'Mocked bot response', source_documents: [{id: 'chap1', title: 'Chapter 1', score: 0.8}] }),
    });
  }
  return Promise.reject(new Error('unknown url'));
}) as jest.Mock;


describe('Chatbot', () => {
  beforeEach(() => {
    // Clear mock calls before each test
    (global.fetch as jest.Mock).mockClear();
  });

  test('renders toggle button', () => {
    render(<Chatbot />);
    expect(screen.getByText('Open Chat')).toBeInTheDocument();
  });

  test('opens and closes chat window', () => {
    render(<Chatbot />);
    fireEvent.click(screen.getByText('Open Chat'));
    expect(screen.getByPlaceholderText('Type your message...')).toBeInTheDocument();
    fireEvent.click(screen.getByText('Close Chat'));
    expect(screen.queryByPlaceholderText('Type your message...')).not.toBeInTheDocument();
  });

  test('sends message and displays bot response', async () => {
    render(<Chatbot />);
    fireEvent.click(screen.getByText('Open Chat'));

    const inputElement = screen.getByPlaceholderText('Type your message...');
    fireEvent.change(inputElement, { target: { value: 'Hello bot' } });
    fireEvent.click(screen.getByText('Send'));

    expect(screen.getByText('Hello bot')).toBeInTheDocument();
    await waitFor(() => {
      expect(screen.getByText('Mocked bot response')).toBeInTheDocument();
    });
    expect(screen.getByText(/Sources:/)).toBeInTheDocument();
    expect(screen.getByText(/- \*\*Chapter 1\*\* \(Score: 0\.80\)/)).toBeInTheDocument();
    expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/chat',
        expect.objectContaining({
            method: 'POST',
            body: JSON.stringify({ query: 'Hello bot' }),
        })
    );
  });

  test('displays error message on fetch failure', async () => {
    (global.fetch as jest.Mock).mockImplementationOnce(() =>
      Promise.resolve({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error',
        json: () => Promise.resolve({}), // Provide a mock json method
      })
    );

    render(<Chatbot />);
    fireEvent.click(screen.getByText('Open Chat'));

    const inputElement = screen.getByPlaceholderText('Type your message...');
    fireEvent.change(inputElement, { target: { value: 'Test error' } });
    fireEvent.click(screen.getByText('Send'));

    await waitFor(() => {
      expect(screen.getByText(/Error: Could not connect to the chatbot./)).toBeInTheDocument();
    });
  });
});
