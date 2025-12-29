import React, { useState, useEffect, useRef } from 'react';
import ReactMarkdown from 'react-markdown';
import rehypeHighlight from 'rehype-highlight';
import './ChatWidget.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [authError, setAuthError] = useState('');
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const [messages, setMessages] = useState([]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Load messages from localStorage on component mount
  useEffect(() => {
    const savedMessages = localStorage.getItem('chatMessages');
    if (savedMessages) {
      try {
        const parsedMessages = JSON.parse(savedMessages);
        setMessages(parsedMessages);
      } catch (e) {
        setMessages([]);
      }
    }
  }, []);

  // Save messages to localStorage whenever messages change
  useEffect(() => {
    if (messages.length > 0) {
      localStorage.setItem('chatMessages', JSON.stringify(messages));
    }
  }, [messages]);

  // Scroll to bottom whenever messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleLogin = async () => {
    try {
      const response = await fetch('http://localhost:8000/login', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          username: username,
          password: password
        })
      });

      if (response.ok) {
        const data = await response.json();
        // Store the token in localStorage
        localStorage.setItem('authToken', data.access_token);
        setIsLoggedIn(true);
        setAuthError('');
        setUsername('');
        setPassword('');

        // Check if this is the first time logging in and add welcome message
        const hasLoggedInBefore = localStorage.getItem('hasLoggedInBefore');
        if (!hasLoggedInBefore) {
          const welcomeMessage = {
            id: Date.now(),
            text: 'Hi! I am your Physical AI Assistant. How can I help you with the courseware today?',
            sender: 'bot',
            isWelcome: true
          };
          setMessages([welcomeMessage]);
          localStorage.setItem('hasLoggedInBefore', 'true');
        }
      } else {
        const errorData = await response.json();
        setAuthError(errorData.detail || 'Login failed');
      }
    } catch (error) {
      setAuthError('Network error. Please try again.');
    }
  };

  const handleLogout = () => {
    localStorage.removeItem('authToken');
    localStorage.removeItem('chatMessages');
    setIsLoggedIn(false);
    setMessages([]);
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const token = localStorage.getItem('authToken');
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({
          message: inputValue,
          history: messages.filter(msg => msg.sender === 'user' || msg.sender === 'bot')
            .map(msg => ({ role: msg.sender === 'user' ? 'user' : 'assistant', content: msg.text }))
        })
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: Date.now() + 1,
          text: data.response,
          sender: 'bot',
          sources: data.context_sources
        };
        setMessages(prev => [...prev, botMessage]);
      } else if (response.status === 401) {
        // Token expired or invalid
        handleLogout();
        setAuthError('Session expired. Please log in again.');
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          text: 'Sorry, I encountered an error. Please try again.',
          sender: 'bot'
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I\'m having trouble connecting. Please check your connection.',
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const copyToClipboard = (text) => {
    navigator.clipboard.writeText(text).then(() => {
      // Optional: Show a temporary notification
      const notification = document.createElement('div');
      notification.textContent = 'Copied!';
      notification.style.cssText = `
        position: fixed;
        top: 50%;
        left: 50%;
        transform: translate(-50%, -50%);
        background: #333;
        color: white;
        padding: 8px 16px;
        border-radius: 4px;
        z-index: 10000;
        font-size: 14px;
      `;
      document.body.appendChild(notification);
      setTimeout(() => {
        document.body.removeChild(notification);
      }, 2000);
    }).catch(err => {
      console.error('Failed to copy: ', err);
    });
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      if (!isLoggedIn) {
        handleLogin();
      } else {
        handleSendMessage();
      }
    }
  };

  return (
    <div className="chat-widget">
      {isOpen ? (
        !isLoggedIn ? (
          <div className="chat-window">
            <div className="chat-header">
              <div className="chat-title">Login Required</div>
              <button
                className="chat-close-btn"
                onClick={() => setIsOpen(false)}
                aria-label="Close chat"
              >
                ×
              </button>
            </div>
            <div className="auth-form">
              <div className="auth-inputs">
                <input
                  type="text"
                  value={username}
                  onChange={(e) => setUsername(e.target.value)}
                  placeholder="Username"
                  className="auth-input"
                />
                <input
                  type="password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  placeholder="Password"
                  className="auth-input"
                  onKeyPress={handleKeyPress}
                />
              </div>
              {authError && <div className="auth-error">{authError}</div>}
              <button
                onClick={handleLogin}
                className="auth-login-btn"
              >
                Login
              </button>
              <div className="auth-hint">
                <small>Default credentials: admin / password</small>
              </div>
            </div>
          </div>
        ) : (
          <div className="chat-window">
            <div className="chat-header">
              <div className="chat-title">Documentation Assistant</div>
              <div className="chat-user-info">
                <span>Hi, {localStorage.getItem('username') || 'User'}!</span>
                <button
                  className="chat-logout-btn"
                  onClick={handleLogout}
                  aria-label="Logout"
                >
                  Logout
                </button>
              </div>
            </div>
            <div className="chat-messages">
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.sender}-message`}
                >
                  <div className="message-text">
  <ReactMarkdown
    rehypePlugins={[rehypeHighlight]}
    components={{
      code({node, inline, className, children, ...props}) {
        const match = /language-(\w+)/.exec(className || '');
        const isCodeBlock = !inline && match;

        return isCodeBlock ? (
          <div className="code-block-wrapper">
            <pre className={className}>
              <code {...props} className={className}>
                {children}
              </code>
            </pre>
            <button
              className="copy-code-btn"
              onClick={() => copyToClipboard(String(children).replace(/\n$/, ''))}
              title="Copy code"
            >
              📋
            </button>
          </div>
        ) : (
          <code className={className} {...props}>
            {children}
          </code>
        );
      }
    }}
  >
    {message.text}
  </ReactMarkdown>
</div>
                  {message.sender === 'bot' && (
                    <button
                      className="copy-message-btn"
                      onClick={() => copyToClipboard(message.text)}
                      title="Copy message"
                    >
                      📋
                    </button>
                  )}
                  {message.sources && message.sources.length > 0 && (
                    <div className="message-sources">
                      <small>Sources: {message.sources.slice(0, 3).join(', ')}</small>
                    </div>
                  )}
                </div>
              ))}
              {isLoading && (
                <div className="message bot-message">
                  <div className="message-text typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>
            <div className="chat-input-area">
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about the documentation..."
                className="chat-input"
                rows="1"
                disabled={isLoading}
              />
              <button
                onClick={handleSendMessage}
                className="chat-send-btn"
                disabled={!inputValue.trim() || isLoading}
                aria-label="Send message"
              >
                Send
              </button>
            </div>
          </div>
        )
      ) : (
        <button
          className="chat-toggle-btn"
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
        >
          💬
        </button>
      )}
    </div>
  );
};

export default ChatWidget;