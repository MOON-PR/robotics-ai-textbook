import React, { useState } from 'react';

interface TranslationButtonProps {
  textToTranslate: string;
}

const TranslationButton: React.FC<TranslationButtonProps> = ({ textToTranslate }) => {
  const [translatedText, setTranslatedText] = useState<string | null>(null);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  const handleTranslate = async () => {
    setLoading(true);
    setError(null);
    try {
      const BACKEND_URL = (typeof window !== 'undefined' && (window as any).__BACKEND_URL__) ? (window as any).__BACKEND_URL__ : (process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000');
      const response = await fetch(`${BACKEND_URL.replace(/\/+$/, '')}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ text: textToTranslate, target_language: 'ur' }),
      });
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setTranslatedText(data.translated_text);
    } catch (e: any) {
      console.error("Translation error:", e);
      setError(`Failed to translate: ${e.message}`);
    } finally {
      setLoading(false);
    }
  };

  const handleReplacePage = () => {
    if (!translatedText) return;
    const el = document.querySelector('.markdown, .theme-doc-markdown');
    if (el) {
      // Replace visible text with the translated content
      (el as HTMLElement).innerText = translatedText;
    } else {
      alert('Could not find page content to replace.');
    }
  };

  return (
    <div style={{ margin: '20px 0', padding: '15px', border: '1px solid #ccc', borderRadius: '8px', backgroundColor: '#f9f9f9' }}>
      <button onClick={handleTranslate} disabled={loading} style={{
        backgroundColor: loading ? '#6c757d' : '#007bff',
        color: 'white',
        padding: '10px 15px',
        border: 'none',
        borderRadius: '5px',
        cursor: loading ? 'not-allowed' : 'pointer',
        fontSize: '16px'
      }}>
        {loading ? 'Translating...' : 'Translate to Urdu'}
      </button>

      {error && <p style={{ color: 'red', marginTop: '10px' }}>{error}</p>}

      {translatedText && (
        <div style={{ marginTop: '15px', paddingTop: '15px', borderTop: '1px dashed #eee' }}>
          <h3>Urdu Translation:</h3>
          <p style={{ fontFamily: 'Noto Naskh Arabic, serif', fontSize: '1.2em', lineHeight: '1.8' }}>
            {translatedText}
          </p>
          <div style={{ marginTop: 12 }}>
            <button onClick={handleReplacePage} style={{ marginRight: 8, padding: '8px 12px', borderRadius: 6, backgroundColor: '#10b981', color: 'white', border: 'none' }}>
              Replace Page With Urdu
            </button>
            <button onClick={() => navigator.clipboard?.writeText(translatedText)} style={{ padding: '8px 12px', borderRadius: 6 }}>
              Copy Translation
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default TranslationButton;
