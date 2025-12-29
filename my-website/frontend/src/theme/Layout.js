import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '../components/ChatWidget';
import { useState, useEffect } from 'react';

// Custom Layout that wraps the original Docusaurus layout
export default function Layout(props) {
  const [showChatWidget, setShowChatWidget] = useState(false);

  // Only show chat widget on client side (not during SSR)
  useEffect(() => {
    setShowChatWidget(true);
  }, []);

  return (
    <>
      <OriginalLayout {...props} />
      {showChatWidget && <ChatWidget />}
    </>
  );
}